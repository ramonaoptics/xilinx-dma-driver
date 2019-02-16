#include "xdma_engine.h"

#include "libxdma.h"
#include "cdev_ctrl.h"

static void engine_reg_dump(struct xdma_engine *engine)
{
	u32 w;

	BUG_ON(!engine);

	w = read_register(&engine->regs->identifier);
	pr_info("%s: ioread32(0x%p) = 0x%08x (id).\n",
		engine->name, &engine->regs->identifier, w);
	w &= BLOCK_ID_MASK;
	if (w != BLOCK_ID_HEAD) {
		pr_info("%s: engine id missing, 0x%08x exp. & 0x%x = 0x%x\n",
			 engine->name, w, BLOCK_ID_MASK, BLOCK_ID_HEAD);
		return;
	}
	/* extra debugging; inspect complete engine set of registers */
	w = read_register(&engine->regs->status);
	pr_info("%s: ioread32(0x%p) = 0x%08x (status).\n",
		engine->name, &engine->regs->status, w);
	w = read_register(&engine->regs->control);
	pr_info("%s: ioread32(0x%p) = 0x%08x (control)\n",
		engine->name, &engine->regs->control, w);
	w = read_register(&engine->sgdma_regs->first_desc_lo);
	pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_lo)\n",
		engine->name, &engine->sgdma_regs->first_desc_lo, w);
	w = read_register(&engine->sgdma_regs->first_desc_hi);
	pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_hi)\n",
		engine->name, &engine->sgdma_regs->first_desc_hi, w);
	w = read_register(&engine->sgdma_regs->first_desc_adjacent);
	pr_info("%s: ioread32(0x%p) = 0x%08x (first_desc_adjacent).\n",
		engine->name, &engine->sgdma_regs->first_desc_adjacent, w);
	w = read_register(&engine->regs->completed_desc_count);
	pr_info("%s: ioread32(0x%p) = 0x%08x (completed_desc_count).\n",
		engine->name, &engine->regs->completed_desc_count, w);
	w = read_register(&engine->regs->interrupt_enable_mask);
	pr_info("%s: ioread32(0x%p) = 0x%08x (interrupt_enable_mask)\n",
		engine->name, &engine->regs->interrupt_enable_mask, w);
}

/**
 * engine_status_read() - read status of SG DMA engine (optionally reset)
 *
 * Stores status in engine->status.
 *
 * @return -1 on failure, status register otherwise
 */
static void engine_status_dump(struct xdma_engine *engine)
{
	u32 v = engine->status;
	char buffer[256];
	char *buf = buffer;
	int len = 0;

	len = sprintf(buf, "SG engine %s status: 0x%08x: ", engine->name, v);

	if ((v & XDMA_STAT_BUSY))
		len += sprintf(buf + len, "BUSY,");
	if ((v & XDMA_STAT_DESC_STOPPED))
		len += sprintf(buf + len, "DESC_STOPPED,");
	if ((v & XDMA_STAT_DESC_COMPLETED))
		len += sprintf(buf + len, "DESC_COMPL,");

	/* common H2C & C2H */
 	if ((v & XDMA_STAT_COMMON_ERR_MASK)) {
		if ((v & XDMA_STAT_ALIGN_MISMATCH))
			len += sprintf(buf + len, "ALIGN_MISMATCH ");
		if ((v & XDMA_STAT_MAGIC_STOPPED))
			len += sprintf(buf + len, "MAGIC_STOPPED ");
		if ((v & XDMA_STAT_INVALID_LEN))
			len += sprintf(buf + len, "INVLIAD_LEN ");
		if ((v & XDMA_STAT_IDLE_STOPPED))
			len += sprintf(buf + len, "IDLE_STOPPED ");
		buf[len - 1] = ',';
	}

	if ((engine->dir == DMA_TO_DEVICE)) {
		/* H2C only */
		if ((v & XDMA_STAT_H2C_R_ERR_MASK)) {
			len += sprintf(buf + len, "R:");
			if ((v & XDMA_STAT_H2C_R_UNSUPP_REQ))
				len += sprintf(buf + len, "UNSUPP_REQ ");
			if ((v & XDMA_STAT_H2C_R_COMPL_ABORT))
				len += sprintf(buf + len, "COMPL_ABORT ");
			if ((v & XDMA_STAT_H2C_R_PARITY_ERR))
				len += sprintf(buf + len, "PARITY ");
			if ((v & XDMA_STAT_H2C_R_HEADER_EP))
				len += sprintf(buf + len, "HEADER_EP ");
			if ((v & XDMA_STAT_H2C_R_UNEXP_COMPL))
				len += sprintf(buf + len, "UNEXP_COMPL ");
			buf[len - 1] = ',';
		}

		if ((v & XDMA_STAT_H2C_W_ERR_MASK)) {
			len += sprintf(buf + len, "W:");
			if ((v & XDMA_STAT_H2C_W_DECODE_ERR))
				len += sprintf(buf + len, "DECODE_ERR ");
			if ((v & XDMA_STAT_H2C_W_SLAVE_ERR))
				len += sprintf(buf + len, "SLAVE_ERR ");
			buf[len - 1] = ',';
		}

	} else {
		/* C2H only */
		if ((v & XDMA_STAT_C2H_R_ERR_MASK)) {
			len += sprintf(buf + len, "R:");
			if ((v & XDMA_STAT_C2H_R_DECODE_ERR))
				len += sprintf(buf + len, "DECODE_ERR ");
			if ((v & XDMA_STAT_C2H_R_SLAVE_ERR))
				len += sprintf(buf + len, "SLAVE_ERR ");
			buf[len - 1] = ',';
		}
	}

	/* common H2C & C2H */
 	if ((v & XDMA_STAT_DESC_ERR_MASK)) {
		len += sprintf(buf + len, "DESC_ERR:");
		if ((v & XDMA_STAT_DESC_UNSUPP_REQ))
			len += sprintf(buf + len, "UNSUPP_REQ ");
		if ((v & XDMA_STAT_DESC_COMPL_ABORT))
			len += sprintf(buf + len, "COMPL_ABORT ");
		if ((v & XDMA_STAT_DESC_PARITY_ERR))
			len += sprintf(buf + len, "PARITY ");
		if ((v & XDMA_STAT_DESC_HEADER_EP))
			len += sprintf(buf + len, "HEADER_EP ");
		if ((v & XDMA_STAT_DESC_UNEXP_COMPL))
			len += sprintf(buf + len, "UNEXP_COMPL ");
		buf[len - 1] = ',';
	}

	buf[len - 1] = '\0';
	pr_info("%s\n", buffer);
}

u32 engine_status_read(struct xdma_engine *engine, bool clear, bool dump)
{
	u32 value;

	BUG_ON(!engine);

	if (dump)
		engine_reg_dump(engine);

	/* read status register */
	if (clear)
		value = engine->status =
			read_register(&engine->regs->status_rc);
	else
		value = engine->status = read_register(&engine->regs->status);

	if (dump)
		engine_status_dump(engine);

	return value;
}

/**
 * engine_stop() - stop an SG DMA engine
 *
 */
void engine_stop(struct xdma_engine *engine)
{
	u32 w;

	BUG_ON(!engine);
	dbg_tfr("engine_stop(engine=%p)\n", engine);

	w = 0;
	w |= (u32)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (u32)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (u32)XDMA_CTRL_IE_READ_ERROR;
	w |= (u32)XDMA_CTRL_IE_DESC_ERROR;

	if (poll_mode) {
		w |= (u32) XDMA_CTRL_POLL_MODE_WB;
	} else {
		w |= (u32)XDMA_CTRL_IE_DESC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_COMPLETED;

		/* Disable IDLE STOPPED for MM */
		if ((engine->streaming && (engine->dir == DMA_FROM_DEVICE)) ||
		    (engine->xdma_perf))
			w |= (u32)XDMA_CTRL_IE_IDLE_STOPPED;
	}

	dbg_tfr("Stopping SG DMA %s engine; writing 0x%08x to 0x%p.\n",
			engine->name, w, (u32 *)&engine->regs->control);
	write_register(w, &engine->regs->control,
			(unsigned long)(&engine->regs->control) -
			(unsigned long)(&engine->regs));
	/* dummy read of status register to flush all previous writes */
	dbg_tfr("engine_stop(%s) done\n", engine->name);
}

static void engine_start_mode_config(struct xdma_engine *engine)
{
	u32 w;

	BUG_ON(!engine);

	/* If a perf test is running, enable the engine interrupts */
	if (engine->xdma_perf) {
		w = XDMA_CTRL_IE_DESC_STOPPED;
		w |= XDMA_CTRL_IE_DESC_COMPLETED;
		w |= XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
		w |= XDMA_CTRL_IE_MAGIC_STOPPED;
		w |= XDMA_CTRL_IE_IDLE_STOPPED;
		w |= XDMA_CTRL_IE_READ_ERROR;
		w |= XDMA_CTRL_IE_DESC_ERROR;

		write_register(w, &engine->regs->interrupt_enable_mask,
			(unsigned long)(&engine->regs->interrupt_enable_mask) -
			(unsigned long)(&engine->regs));
	}

	/* write control register of SG DMA engine */
	w = (u32)XDMA_CTRL_RUN_STOP;
	w |= (u32)XDMA_CTRL_IE_READ_ERROR;
	w |= (u32)XDMA_CTRL_IE_DESC_ERROR;
	w |= (u32)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (u32)XDMA_CTRL_IE_MAGIC_STOPPED;

	if (poll_mode) {
		w |= (u32)XDMA_CTRL_POLL_MODE_WB;
	} else {
		w |= (u32)XDMA_CTRL_IE_DESC_STOPPED;
		w |= (u32)XDMA_CTRL_IE_DESC_COMPLETED;

		if ((engine->streaming && (engine->dir == DMA_FROM_DEVICE)) ||
		    (engine->xdma_perf))
			w |= (u32)XDMA_CTRL_IE_IDLE_STOPPED;

		/* set non-incremental addressing mode */
		if (engine->non_incr_addr)
			w |= (u32)XDMA_CTRL_NON_INCR_ADDR;
	}

	dbg_tfr("iowrite32(0x%08x to 0x%p) (control)\n", w,
			(void *)&engine->regs->control);
	/* start the engine */
	write_register(w, &engine->regs->control,
			(unsigned long)(&engine->regs->control) -
			(unsigned long)(&engine->regs));

	/* dummy read of status register to flush all previous writes */
	w = read_register(&engine->regs->status);
	dbg_tfr("ioread32(0x%p) = 0x%08x (dummy read flushes writes).\n",
			&engine->regs->status, w);
}

/**
 * engine_start() - start an idle engine with its first transfer on queue
 *
 * The engine will run and process all transfers that are queued using
 * transfer_queue() and thus have their descriptor lists chained.
 *
 * During the run, new transfers will be processed if transfer_queue() has
 * chained the descriptors before the hardware fetches the last descriptor.
 * A transfer that was chained too late will invoke a new run of the engine
 * initiated from the engine_service() routine.
 *
 * The engine must be idle and at least one transfer must be queued.
 * This function does not take locks; the engine spinlock must already be
 * taken.
 *
 */
struct xdma_transfer *engine_start(struct xdma_engine *engine)
{
	struct xdma_transfer *transfer;
	u32 w;
	int extra_adj = 0;

	/* engine must be idle */
	BUG_ON(engine->running);
	/* engine transfer queue must not be empty */
	BUG_ON(list_empty(&engine->transfer_list));
	/* inspect first transfer queued on the engine */
	transfer = list_entry(engine->transfer_list.next, struct xdma_transfer,
				entry);
	BUG_ON(!transfer);

	/* engine is no longer shutdown */
	engine->shutdown = ENGINE_SHUTDOWN_NONE;

	dbg_tfr("engine_start(%s): transfer=0x%p.\n", engine->name, transfer);

	/* initialize number of descriptors of dequeued transfers */
	engine->desc_dequeued = 0;

	/* write lower 32-bit of bus address of transfer first descriptor */
	w = cpu_to_le32(PCI_DMA_L(transfer->desc_bus));
	dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_lo)\n", w,
			(void *)&engine->sgdma_regs->first_desc_lo);
	write_register(w, &engine->sgdma_regs->first_desc_lo,
			(unsigned long)(&engine->sgdma_regs->first_desc_lo) -
			(unsigned long)(&engine->sgdma_regs));
	/* write upper 32-bit of bus address of transfer first descriptor */
	w = cpu_to_le32(PCI_DMA_H(transfer->desc_bus));
	dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_hi)\n", w,
			(void *)&engine->sgdma_regs->first_desc_hi);
	write_register(w, &engine->sgdma_regs->first_desc_hi,
			(unsigned long)(&engine->sgdma_regs->first_desc_hi) -
			(unsigned long)(&engine->sgdma_regs));

	if (transfer->desc_adjacent > 0) {
		extra_adj = transfer->desc_adjacent - 1;
		if (extra_adj > MAX_EXTRA_ADJ)
			extra_adj = MAX_EXTRA_ADJ;
	}
	dbg_tfr("iowrite32(0x%08x to 0x%p) (first_desc_adjacent)\n",
		extra_adj, (void *)&engine->sgdma_regs->first_desc_adjacent);
	write_register(extra_adj, &engine->sgdma_regs->first_desc_adjacent,
			(unsigned long)(&engine->sgdma_regs->first_desc_adjacent) -
			(unsigned long)(&engine->sgdma_regs));

	dbg_tfr("ioread32(0x%p) (dummy read flushes writes).\n",
		&engine->regs->status);
	mmiowb();

	engine_start_mode_config(engine);

	engine_status_read(engine, 0, 0);

	dbg_tfr("%s engine 0x%p now running\n", engine->name, engine);
	/* remember the engine is running */
	engine->running = 1;
	return transfer;
}

/**
 * engine_service() - service an SG DMA engine
 *
 * must be called with engine->lock already acquired
 *
 * @engine pointer to struct xdma_engine
 *
 */
void engine_service_shutdown(struct xdma_engine *engine)
{
	/* if the engine stopped with RUN still asserted, de-assert RUN now */
	dbg_tfr("engine just went idle, resetting RUN_STOP.\n");
	engine_stop(engine);
	engine->running = 0;

	/* awake task on engine's shutdown wait queue */
	wake_up_interruptible(&engine->shutdown_wq);
}

struct xdma_transfer *engine_transfer_completion(struct xdma_engine *engine,
		struct xdma_transfer *transfer)
{
	BUG_ON(!engine);

	if (unlikely(!transfer)) {
		pr_info("%s: xfer empty.\n", engine->name);
		return NULL;
	}

	/* synchronous I/O? */
	/* awake task on transfer's wait queue */
	wake_up_interruptible(&transfer->wq);

	return transfer;
}

struct xdma_transfer *engine_service_transfer_list(struct xdma_engine *engine,
		struct xdma_transfer *transfer, u32 *pdesc_completed)
{
	BUG_ON(!engine);
	BUG_ON(!pdesc_completed);

	if (unlikely(!transfer)) {
		pr_info("%s xfer empty, pdesc completed %u.\n",
			engine->name, *pdesc_completed);
		return NULL;
	}

	/*
	 * iterate over all the transfers completed by the engine,
	 * except for the last (i.e. use > instead of >=).
	 */
	while (transfer && (!transfer->cyclic) &&
		(*pdesc_completed > transfer->desc_num)) {
		/* remove this transfer from pdesc_completed */
		*pdesc_completed -= transfer->desc_num;
		dbg_tfr("%s engine completed non-cyclic xfer 0x%p (%d desc)\n",
			engine->name, transfer, transfer->desc_num);
		/* remove completed transfer from list */
		list_del(engine->transfer_list.next);
		/* add to dequeued number of descriptors during this run */
		engine->desc_dequeued += transfer->desc_num;
		/* mark transfer as succesfully completed */
		transfer->state = TRANSFER_STATE_COMPLETED;

		/* Complete transfer - sets transfer to NULL if an async
		 * transfer has completed */
		transfer = engine_transfer_completion(engine, transfer);

		/* if exists, get the next transfer on the list */
		if (!list_empty(&engine->transfer_list)) {
			transfer = list_entry(engine->transfer_list.next,
					struct xdma_transfer, entry);
			dbg_tfr("Non-completed transfer %p\n", transfer);
		} else {
			/* no further transfers? */
			transfer = NULL;
		}
	}

	return transfer;
}

static void engine_err_handle(struct xdma_engine *engine,
		struct xdma_transfer *transfer, u32 desc_completed)
{
	u32 value;

	/*
	 * The BUSY bit is expected to be clear now but older HW has a race
	 * condition which could cause it to be still set.  If it's set, re-read
	 * and check again.  If it's still set, log the issue.
	 */
	if (engine->status & XDMA_STAT_BUSY) {
		value = read_register(&engine->regs->status);
		if ((value & XDMA_STAT_BUSY) && printk_ratelimit())
			pr_info("%s has errors but is still BUSY\n",
				engine->name);
	}

	if (printk_ratelimit()) {
		pr_info("%s, s 0x%x, aborted xfer 0x%p, cmpl %d/%d\n",
			engine->name, engine->status, transfer, desc_completed,
			transfer->desc_num);
	}

	/* mark transfer as failed */
	transfer->state = TRANSFER_STATE_FAILED;
	engine_stop(engine);
}

struct xdma_transfer *engine_service_final_transfer(struct xdma_engine *engine,
			struct xdma_transfer *transfer, u32 *pdesc_completed)
{
	BUG_ON(!engine);
	BUG_ON(!pdesc_completed);

	/* inspect the current transfer */
	if (unlikely(!transfer)) {
		pr_info("%s xfer empty, pdesc completed %u.\n",
			engine->name, *pdesc_completed);
		return NULL;
	} else {
		if (((engine->dir == DMA_FROM_DEVICE) &&
		     (engine->status & XDMA_STAT_C2H_ERR_MASK)) ||
		    ((engine->dir == DMA_TO_DEVICE) &&
		     (engine->status & XDMA_STAT_H2C_ERR_MASK))) {
			pr_info("engine %s, status error 0x%x.\n",
				engine->name, engine->status);
			engine_status_dump(engine);
			engine_err_handle(engine, transfer, *pdesc_completed);
			goto transfer_del;
		}

		if (engine->status & XDMA_STAT_BUSY)
			pr_debug("engine %s is unexpectedly busy - ignoring\n",
				engine->name);

		/* the engine stopped on current transfer? */
		if (*pdesc_completed < transfer->desc_num) {
			transfer->state = TRANSFER_STATE_FAILED;
			pr_info("%s, xfer 0x%p, stopped half-way, %d/%d.\n",
				engine->name, transfer, *pdesc_completed,
				transfer->desc_num);
		} else {
			dbg_tfr("engine %s completed transfer\n", engine->name);
			dbg_tfr("Completed transfer ID = 0x%p\n", transfer);
			dbg_tfr("*pdesc_completed=%d, transfer->desc_num=%d",
				*pdesc_completed, transfer->desc_num);

			if (!transfer->cyclic) {
				/*
				 * if the engine stopped on this transfer,
				 * it should be the last
				 */
				WARN_ON(*pdesc_completed > transfer->desc_num);
			}
			/* mark transfer as succesfully completed */
			transfer->state = TRANSFER_STATE_COMPLETED;
		}

transfer_del:
		/* remove completed transfer from list */
		list_del(engine->transfer_list.next);
		/* add to dequeued number of descriptors during this run */
		engine->desc_dequeued += transfer->desc_num;

		/*
		 * Complete transfer - sets transfer to NULL if an asynchronous
		 * transfer has completed
		 */
		transfer = engine_transfer_completion(engine, transfer);
	}

	return transfer;
}

static void engine_service_perf(struct xdma_engine *engine, u32 desc_completed)
{
	BUG_ON(!engine);

	/* performance measurement is running? */
	if (engine->xdma_perf) {
		/* a descriptor was completed? */
		if (engine->status & XDMA_STAT_DESC_COMPLETED) {
			engine->xdma_perf->iterations = desc_completed;
			dbg_perf("transfer->xdma_perf->iterations=%d\n",
				engine->xdma_perf->iterations);
		}

		/* a descriptor stopped the engine? */
		if (engine->status & XDMA_STAT_DESC_STOPPED) {
			engine->xdma_perf->stopped = 1;
			/*
			 * wake any XDMA_PERF_IOCTL_STOP waiting for
			 * the performance run to finish
			 */
			wake_up_interruptible(&engine->xdma_perf_wq);
			dbg_perf("transfer->xdma_perf stopped\n");
		}
	}
}

void engine_transfer_dequeue(struct xdma_engine *engine)
{
	struct xdma_transfer *transfer;

	BUG_ON(!engine);

	/* pick first transfer on the queue (was submitted to the engine) */
	transfer = list_entry(engine->transfer_list.next, struct xdma_transfer,
		entry);
	if (!transfer || transfer != &engine->cyclic_req->xfer) {
		pr_info("%s, xfer 0x%p != 0x%p.\n",
			engine->name, transfer, &engine->cyclic_req->xfer);
		return;
	}
	dbg_tfr("%s engine completed cyclic transfer 0x%p (%d desc).\n",
		engine->name, transfer, transfer->desc_num);
	/* remove completed transfer from list */
	list_del(engine->transfer_list.next);
}

static int engine_ring_process(struct xdma_engine *engine)
{
	struct xdma_result *result;
	int start;
	int eop_count = 0;

	BUG_ON(!engine);
	result = engine->cyclic_result;
	BUG_ON(!result);

	/* where we start receiving in the ring buffer */
	start = engine->rx_tail;

	/* iterate through all newly received RX result descriptors */
	dbg_tfr("%s, result %d, 0x%x, len 0x%x.\n",
		engine->name, engine->rx_tail, result[engine->rx_tail].status,
		result[engine->rx_tail].length);
	while (result[engine->rx_tail].status && !engine->rx_overrun) {
		/* EOP bit set in result? */
		if (result[engine->rx_tail].status & RX_STATUS_EOP){
			eop_count++;
		}

		/* increment tail pointer */
		engine->rx_tail = (engine->rx_tail + 1) % CYCLIC_RX_PAGES_MAX;

		dbg_tfr("%s, head %d, tail %d, 0x%x, len 0x%x.\n",
			engine->name, engine->rx_head, engine->rx_tail,
			result[engine->rx_tail].status,
			result[engine->rx_tail].length);

		/* overrun? */
		if (engine->rx_tail == engine->rx_head) {
			dbg_tfr("%s: overrun\n", engine->name);
			/* flag to user space that overrun has occurred */
			engine->rx_overrun = 1;
		}
	}

	return eop_count;
}

static int engine_service_cyclic_polled(struct xdma_engine *engine)
{
	int eop_count = 0;
	int rc = 0;
	struct xdma_poll_wb *writeback_data;
	u32 sched_limit = 0;

	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	writeback_data = (struct xdma_poll_wb *)engine->poll_mode_addr_virt;

	while (eop_count == 0) {
		if (sched_limit != 0) {
			if ((sched_limit % NUM_POLLS_PER_SCHED) == 0)
				schedule();
		}
		sched_limit++;

		/* Monitor descriptor writeback address for errors */
		if ((writeback_data->completed_desc_count) & WB_ERR_MASK) {
			rc = -1;
			break;
		}

		eop_count = engine_ring_process(engine);
	}

	if (eop_count == 0) {
		engine_status_read(engine, 1, 0);
		if ((engine->running) && !(engine->status & XDMA_STAT_BUSY)) {
			/* transfers on queue? */
			if (!list_empty(&engine->transfer_list))
				engine_transfer_dequeue(engine);

			engine_service_shutdown(engine);
		}
	}

	return rc;
}

static int engine_service_cyclic_interrupt(struct xdma_engine *engine)
{
	int eop_count = 0;
	struct xdma_transfer *xfer;

	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	engine_status_read(engine, 1, 0);

	eop_count = engine_ring_process(engine);
	/*
	 * wake any reader on EOP, as one or more packets are now in
	 * the RX buffer
	 */
	xfer = &engine->cyclic_req->xfer;
	if(enable_credit_mp){
		if (eop_count > 0) {
			//engine->eop_found = 1;
		}
		wake_up_interruptible(&xfer->wq);
	}else{
		if (eop_count > 0) {
			/* awake task on transfer's wait queue */
			dbg_tfr("wake_up_interruptible() due to %d EOP's\n", eop_count);
			engine->eop_found = 1;
			wake_up_interruptible(&xfer->wq);
		}
	}

	/* engine was running but is no longer busy? */
	if ((engine->running) && !(engine->status & XDMA_STAT_BUSY)) {
		/* transfers on queue? */
		if (!list_empty(&engine->transfer_list))
			engine_transfer_dequeue(engine);

		engine_service_shutdown(engine);
	}

	return 0;
}

/* must be called with engine->lock already acquired */
static int engine_service_cyclic(struct xdma_engine *engine)
{
	int rc = 0;

	dbg_tfr("engine_service_cyclic()");

	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	if (poll_mode)
		rc = engine_service_cyclic_polled(engine);
	else
		rc = engine_service_cyclic_interrupt(engine);

	return rc;
}


static void engine_service_resume(struct xdma_engine *engine)
{
	struct xdma_transfer *transfer_started;

	BUG_ON(!engine);

	/* engine stopped? */
	if (!engine->running) {
		/* in the case of shutdown, let it finish what's in the Q */
		if (!list_empty(&engine->transfer_list)) {
			/* (re)start engine */
			transfer_started = engine_start(engine);
			pr_info("re-started %s engine with pending xfer 0x%p\n",
				engine->name, transfer_started);
		/* engine was requested to be shutdown? */
		} else if (engine->shutdown & ENGINE_SHUTDOWN_REQUEST) {
			engine->shutdown |= ENGINE_SHUTDOWN_IDLE;
			/* awake task on engine's shutdown wait queue */
			wake_up_interruptible(&engine->shutdown_wq);
		} else {
			dbg_tfr("no pending transfers, %s engine stays idle.\n",
				engine->name);
		}
	} else {
		/* engine is still running? */
		if (list_empty(&engine->transfer_list)) {
			pr_warn("no queued transfers but %s engine running!\n",
				engine->name);
			WARN_ON(1);
		}
	}
}

/**
 * engine_service() - service an SG DMA engine
 *
 * must be called with engine->lock already acquired
 *
 * @engine pointer to struct xdma_engine
 *
 */
static int engine_service(struct xdma_engine *engine, int desc_writeback)
{
	struct xdma_transfer *transfer = NULL;
	u32 desc_count = desc_writeback & WB_COUNT_MASK;
	u32 err_flag = desc_writeback & WB_ERR_MASK;
	int rv = 0;
	struct xdma_poll_wb *wb_data;

	BUG_ON(!engine);

	/* If polling detected an error, signal to the caller */
	if (err_flag)
		rv = -1;

	/* Service the engine */
	if (!engine->running) {
		dbg_tfr("Engine was not running!!! Clearing status\n");
		engine_status_read(engine, 1, 0);
		return 0;
	}

	/*
	 * If called by the ISR or polling detected an error, read and clear
	 * engine status. For polled mode descriptor completion, this read is
	 * unnecessary and is skipped to reduce latency
	 */
	if ((desc_count == 0) || (err_flag != 0))
		engine_status_read(engine, 1, 0);

	/*
	 * engine was running but is no longer busy, or writeback occurred,
	 * shut down
	 */
	if ((engine->running && !(engine->status & XDMA_STAT_BUSY)) ||
		(desc_count != 0))
		engine_service_shutdown(engine);

	/*
	 * If called from the ISR, or if an error occurred, the descriptor
	 * count will be zero.  In this scenario, read the descriptor count
	 * from HW.  In polled mode descriptor completion, this read is
	 * unnecessary and is skipped to reduce latency
	 */
	if (!desc_count)
		desc_count = read_register(&engine->regs->completed_desc_count);
	dbg_tfr("desc_count = %d\n", desc_count);

	/* transfers on queue? */
	if (!list_empty(&engine->transfer_list)) {
		/* pick first transfer on queue (was submitted to the engine) */
		transfer = list_entry(engine->transfer_list.next,
				struct xdma_transfer, entry);

		dbg_tfr("head of queue transfer 0x%p has %d descriptors\n",
			transfer, (int)transfer->desc_num);

		dbg_tfr("Engine completed %d desc, %d not yet dequeued\n",
			(int)desc_count,
			(int)desc_count - engine->desc_dequeued);

		engine_service_perf(engine, desc_count);
	}

	/* account for already dequeued transfers during this engine run */
	desc_count -= engine->desc_dequeued;

	/* Process all but the last transfer */
	transfer = engine_service_transfer_list(engine, transfer, &desc_count);

	/*
	 * Process final transfer - includes checks of number of descriptors to
	 * detect faulty completion
	 */
	transfer = engine_service_final_transfer(engine, transfer, &desc_count);

	/* Before starting engine again, clear the writeback data */
	if (poll_mode) {
		wb_data = (struct xdma_poll_wb *)engine->poll_mode_addr_virt;
		wb_data->completed_desc_count = 0;
	}

	/* Restart the engine following the servicing */
	engine_service_resume(engine);

	return 0;
}

/* engine_service_work */
static void engine_service_work(struct work_struct *work)
{
	struct xdma_engine *engine;
	unsigned long flags;

	engine = container_of(work, struct xdma_engine, work);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	/* lock the engine */
	spin_lock_irqsave(&engine->lock, flags);

	dbg_tfr("engine_service() for %s engine %p\n",
		engine->name, engine);
	if (engine->cyclic_req)
		engine_service_cyclic(engine);
	else
		engine_service(engine, 0);

	/* re-enable interrupts for this engine */
	if (engine->xdev->msix_enabled){
		write_register(engine->interrupt_enable_mask_value,
			       &engine->regs->interrupt_enable_mask_w1s,
			(unsigned long)(&engine->regs->interrupt_enable_mask_w1s) -
			(unsigned long)(&engine->regs));
	} else
		channel_interrupts_enable(engine->xdev, engine->irq_bitmask);

	/* unlock the engine */
	spin_unlock_irqrestore(&engine->lock, flags);
}

static u32 engine_service_wb_monitor(struct xdma_engine *engine,
	u32 expected_wb)
{
	struct xdma_poll_wb *wb_data;
	u32 desc_wb = 0;
	u32 sched_limit = 0;
	unsigned long timeout;

	BUG_ON(!engine);
	wb_data = (struct xdma_poll_wb *)engine->poll_mode_addr_virt;

	/*
 	 * Poll the writeback location for the expected number of
 	 * descriptors / error events This loop is skipped for cyclic mode,
 	 * where the expected_desc_count passed in is zero, since it cannot be
 	 * determined before the function is called
 	 */

	timeout = jiffies + (POLL_TIMEOUT_SECONDS * HZ);
	while (expected_wb != 0) {
		desc_wb = wb_data->completed_desc_count;

		if (desc_wb & WB_ERR_MASK)
			break;
		else if (desc_wb == expected_wb)
			break;

		/* RTO - prevent system from hanging in polled mode */
		if (time_after(jiffies, timeout)) {
			dbg_tfr("Polling timeout occurred");
			dbg_tfr("desc_wb = 0x%08x, expected 0x%08x\n", desc_wb,
				expected_wb);
			if ((desc_wb & WB_COUNT_MASK) > expected_wb)
				desc_wb = expected_wb | WB_ERR_MASK;

			break;
		}

		/*
 		 * Define NUM_POLLS_PER_SCHED to limit how much time is spent
 		 * in the scheduler
 		 */

		if (sched_limit != 0) {
			if ((sched_limit % NUM_POLLS_PER_SCHED) == 0)
				schedule();
		}
		sched_limit++;
	}

	return desc_wb;
}

int engine_service_poll(struct xdma_engine *engine, u32 expected_desc_count)
{
	struct xdma_poll_wb *writeback_data;
	u32 desc_wb = 0;
	unsigned long flags;
	int rv = 0;

	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	writeback_data = (struct xdma_poll_wb *)engine->poll_mode_addr_virt;

	if ((expected_desc_count & WB_COUNT_MASK) != expected_desc_count) {
		dbg_tfr("Queued descriptor count is larger than supported\n");
		return -1;
	}

	/*
 	 * Poll the writeback location for the expected number of
 	 * descriptors / error events This loop is skipped for cyclic mode,
 	 * where the expected_desc_count passed in is zero, since it cannot be
 	 * determined before the function is called
 	 */

	desc_wb = engine_service_wb_monitor(engine, expected_desc_count);

	spin_lock_irqsave(&engine->lock, flags);
	dbg_tfr("%s service.\n", engine->name);
	if (engine->cyclic_req) {
		rv = engine_service_cyclic(engine);
	} else {
		rv = engine_service(engine, desc_wb);
	}
	spin_unlock_irqrestore(&engine->lock, flags);

	return rv;
}

static void engine_alignments(struct xdma_engine *engine)
{
	u32 w;
	u32 align_bytes;
	u32 granularity_bytes;
	u32 address_bits;

	w = read_register(&engine->regs->alignments);
	dbg_init("engine %p name %s alignments=0x%08x\n", engine,
		engine->name, (int)w);

	/* RTO  - add some macros to extract these fields */
	align_bytes = (w & 0x00ff0000U) >> 16;
	granularity_bytes = (w & 0x0000ff00U) >> 8;
	address_bits = (w & 0x000000ffU);

	dbg_init("align_bytes = %d\n", align_bytes);
	dbg_init("granularity_bytes = %d\n", granularity_bytes);
	dbg_init("address_bits = %d\n", address_bits);

	if (w) {
		engine->addr_align = align_bytes;
		engine->len_granularity = granularity_bytes;
		engine->addr_bits = address_bits;
	} else {
		/* Some default values if alignments are unspecified */
		engine->addr_align = 1;
		engine->len_granularity = 1;
		engine->addr_bits = 64;
	}
}

static void engine_free_resource(struct xdma_engine *engine)
{
	struct xdma_dev *xdev = engine->xdev;

	/* Release memory use for descriptor writebacks */
	if (engine->poll_mode_addr_virt) {
		dbg_sg("Releasing memory for descriptor writeback\n");
		dma_free_coherent(&xdev->pdev->dev,
				sizeof(struct xdma_poll_wb),
				engine->poll_mode_addr_virt,
				engine->poll_mode_bus);
		dbg_sg("Released memory for descriptor writeback\n");
		engine->poll_mode_addr_virt = NULL;
	}

	if (engine->desc) {
		dbg_init("device %s, engine %s pre-alloc desc 0x%p,0x%llx.\n",
			dev_name(&xdev->pdev->dev), engine->name,
			engine->desc, engine->desc_bus);
		dma_free_coherent(&xdev->pdev->dev,
			XDMA_TRANSFER_MAX_DESC * sizeof(struct xdma_desc),
			engine->desc, engine->desc_bus);
		engine->desc = NULL;
	}

	if (engine->cyclic_result) {
		dma_free_coherent(&xdev->pdev->dev,
			CYCLIC_RX_PAGES_MAX * sizeof(struct xdma_result),
			engine->cyclic_result, engine->cyclic_result_bus);
		engine->cyclic_result = NULL;
	}
}

void engine_destroy(struct xdma_dev *xdev, struct xdma_engine *engine)
{
	BUG_ON(!xdev);
	BUG_ON(!engine);

	dbg_sg("Shutting down engine %s%d", engine->name, engine->channel);

	/* Disable interrupts to stop processing new events during shutdown */
	write_register(0x0, &engine->regs->interrupt_enable_mask,
			(unsigned long)(&engine->regs->interrupt_enable_mask) -
			(unsigned long)(&engine->regs));

	if (enable_credit_mp && engine->streaming &&
		engine->dir == DMA_FROM_DEVICE) {
		u32 reg_value = (0x1 << engine->channel) << 16;
		struct sgdma_common_regs *reg = (struct sgdma_common_regs *)
				(xdev->bar[xdev->config_bar_idx] +
				 (0x6*TARGET_SPACING));
		write_register(reg_value, &reg->credit_mode_enable_w1c, 0);
	}

	/* Release memory use for descriptor writebacks */
	engine_free_resource(engine);

	memset(engine, 0, sizeof(struct xdma_engine));
	/* Decrement the number of engines available */
	xdev->engines_num--;
}

int engine_addrmode_set(struct xdma_engine *engine, unsigned long arg)
{
	int rv;
	unsigned long dst;
	u32 w = XDMA_CTRL_NON_INCR_ADDR;

	dbg_perf("IOCTL_XDMA_ADDRMODE_SET\n");
	rv = get_user(dst, (int __user *)arg);

	if (rv == 0) {
		engine->non_incr_addr = !!dst;
		if (engine->non_incr_addr)
			write_register(w, &engine->regs->control_w1s,
				(unsigned long)(&engine->regs->control_w1s) -
				(unsigned long)(&engine->regs));
		else
			write_register(w, &engine->regs->control_w1c,
				(unsigned long)(&engine->regs->control_w1c) -
				(unsigned long)(&engine->regs));
	}
	engine_alignments(engine);

	return rv;
}


static int engine_alloc_resource(struct xdma_engine *engine)
{
	struct xdma_dev *xdev = engine->xdev;

	engine->desc = dma_alloc_coherent(&xdev->pdev->dev,
			XDMA_TRANSFER_MAX_DESC * sizeof(struct xdma_desc),
			&engine->desc_bus, GFP_KERNEL);
	if (!engine->desc) {
		pr_warn("dev %s, %s pre-alloc desc OOM.\n",
			dev_name(&xdev->pdev->dev), engine->name);
		goto err_out;
	}

	if (poll_mode) {
		engine->poll_mode_addr_virt = dma_alloc_coherent(
					&xdev->pdev->dev,
					sizeof(struct xdma_poll_wb),
					&engine->poll_mode_bus, GFP_KERNEL);
		if (!engine->poll_mode_addr_virt) {
			pr_warn("%s, %s poll pre-alloc writeback OOM.\n",
				dev_name(&xdev->pdev->dev), engine->name);
			goto err_out;
		}
	}

	if (engine->streaming && engine->dir == DMA_FROM_DEVICE) {
		engine->cyclic_result = dma_alloc_coherent(&xdev->pdev->dev,
			CYCLIC_RX_PAGES_MAX * sizeof(struct xdma_result),
			&engine->cyclic_result_bus, GFP_KERNEL);

		if (!engine->cyclic_result) {
			pr_warn("%s, %s pre-alloc result OOM.\n",
				dev_name(&xdev->pdev->dev), engine->name);
			goto err_out;
		}
	}

	return 0;

err_out:
	engine_free_resource(engine);
	return -ENOMEM;
}

static int engine_writeback_setup(struct xdma_engine *engine)
{
	u32 w;
	struct xdma_dev *xdev;
	struct xdma_poll_wb *writeback;

	BUG_ON(!engine);
	xdev = engine->xdev;
	BUG_ON(!xdev);

	/*
	 * RTO - doing the allocation per engine is wasteful since a full page
	 * is allocated each time - better to allocate one page for the whole
	 * device during probe() and set per-engine offsets here
	 */
	writeback = (struct xdma_poll_wb *)engine->poll_mode_addr_virt;
	writeback->completed_desc_count = 0;

	dbg_init("Setting writeback location to 0x%llx for engine %p",
		engine->poll_mode_bus, engine);
	w = cpu_to_le32(PCI_DMA_L(engine->poll_mode_bus));
	write_register(w, &engine->regs->poll_mode_wb_lo,
			(unsigned long)(&engine->regs->poll_mode_wb_lo) -
			(unsigned long)(&engine->regs));
	w = cpu_to_le32(PCI_DMA_H(engine->poll_mode_bus));
	write_register(w, &engine->regs->poll_mode_wb_hi,
			(unsigned long)(&engine->regs->poll_mode_wb_hi) -
			(unsigned long)(&engine->regs));

	return 0;
}

/* engine_create() - Create an SG DMA engine bookkeeping data structure
 *
 * An SG DMA engine consists of the resources for a single-direction transfer
 * queue; the SG DMA hardware, the software queue and interrupt handling.
 *
 * @dev Pointer to pci_dev
 * @offset byte address offset in BAR[xdev->config_bar_idx] resource for the
 * SG DMA * controller registers.
 * @dir: DMA_TO/FROM_DEVICE
 * @streaming Whether the engine is attached to AXI ST (rather than MM)
 */
int engine_init_regs(struct xdma_engine *engine)
{
	u32 reg_value;
	int rv = 0;

	write_register(XDMA_CTRL_NON_INCR_ADDR, &engine->regs->control_w1c,
			(unsigned long)(&engine->regs->control_w1c) -
			(unsigned long)(&engine->regs));

	engine_alignments(engine);

	/* Configure error interrupts by default */
	reg_value = XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	reg_value |= XDMA_CTRL_IE_MAGIC_STOPPED;
	reg_value |= XDMA_CTRL_IE_MAGIC_STOPPED;
	reg_value |= XDMA_CTRL_IE_READ_ERROR;
	reg_value |= XDMA_CTRL_IE_DESC_ERROR;

	/* if using polled mode, configure writeback address */
	if (poll_mode) {
		rv = engine_writeback_setup(engine);
		if (rv) {
			dbg_init("%s descr writeback setup failed.\n",
				engine->name);
			goto fail_wb;
		}
	} else {
		/* enable the relevant completion interrupts */
		reg_value |= XDMA_CTRL_IE_DESC_STOPPED;
		reg_value |= XDMA_CTRL_IE_DESC_COMPLETED;

		if (engine->streaming && engine->dir == DMA_FROM_DEVICE)
			reg_value |= XDMA_CTRL_IE_IDLE_STOPPED;
	}

	/* Apply engine configurations */
	write_register(reg_value, &engine->regs->interrupt_enable_mask,
			(unsigned long)(&engine->regs->interrupt_enable_mask) -
			(unsigned long)(&engine->regs));

	engine->interrupt_enable_mask_value = reg_value;

	/* only enable credit mode for AXI-ST C2H */
	if (enable_credit_mp && engine->streaming &&
		engine->dir == DMA_FROM_DEVICE) {

		struct xdma_dev *xdev = engine->xdev;
		u32 reg_value = (0x1 << engine->channel) << 16;
		struct sgdma_common_regs *reg = (struct sgdma_common_regs *)
				(xdev->bar[xdev->config_bar_idx] +
				 (0x6*TARGET_SPACING));

		write_register(reg_value, &reg->credit_mode_enable_w1s, 0);
	}

	return 0;

fail_wb:
	return rv;
}

int engine_init(struct xdma_engine *engine, struct xdma_dev *xdev,
			int offset, enum dma_data_direction dir, int channel)
{
	int rv;
	u32 val;

	dbg_init("channel %d, offset 0x%x, dir %d.\n", channel, offset, dir);

	/* set magic */
	engine->magic = MAGIC_ENGINE;

	engine->channel = channel;

	/* engine interrupt request bit */
	engine->irq_bitmask = (1 << XDMA_ENG_IRQ_NUM) - 1;
	engine->irq_bitmask <<= (xdev->engines_num * XDMA_ENG_IRQ_NUM);
	engine->bypass_offset = xdev->engines_num * BYPASS_MODE_SPACING;

	/* parent */
	engine->xdev = xdev;
	/* register address */
	engine->regs = (xdev->bar[xdev->config_bar_idx] + offset);
	engine->sgdma_regs = xdev->bar[xdev->config_bar_idx] + offset +
				SGDMA_OFFSET_FROM_CHANNEL;
	val = read_register(&engine->regs->identifier);
	if (val & 0x8000U)
		engine->streaming = 1;

	/* remember SG DMA direction */
	engine->dir = dir;
	sprintf(engine->name, "%d-%s%d-%s", xdev->idx,
		(dir == DMA_TO_DEVICE) ? "H2C" : "C2H", channel,
		engine->streaming ? "ST" : "MM");

	dbg_init("engine %p name %s irq_bitmask=0x%08x\n", engine, engine->name,
		(int)engine->irq_bitmask);

	/* initialize the deferred work for transfer completion */
	INIT_WORK(&engine->work, engine_service_work);

	if (dir == DMA_TO_DEVICE)
		xdev->mask_irq_h2c |= engine->irq_bitmask;
	else
		xdev->mask_irq_c2h |= engine->irq_bitmask;
	xdev->engines_num++;

	rv = engine_alloc_resource(engine);
	if (rv)
		return rv;

	rv = engine_init_regs(engine);
	if (rv)
		return rv;

	return 0;
}
