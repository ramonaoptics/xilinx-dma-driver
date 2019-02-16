#ifndef XDMA_ENGINE_H
#define XDMA_ENGINE_H
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <linux/types.h>
#endif
#include <linux/dma-mapping.h>


enum shutdown_state {
	ENGINE_SHUTDOWN_NONE = 0,	/* No shutdown in progress */
	ENGINE_SHUTDOWN_REQUEST = 1,	/* engine requested to shutdown */
	ENGINE_SHUTDOWN_IDLE = 2	/* engine has shutdown and is idle */
};

struct xdma_engine {
	unsigned long magic;	/* structure ID for sanity checks */
	struct xdma_dev *xdev;	/* parent device */
	char name[5];		/* name of this engine */
	int version;		/* version of this engine */
	//dev_t cdevno;		/* character device major:minor */
	//struct cdev cdev;	/* character device (embedded struct) */

	/* HW register address offsets */
	struct engine_regs *regs;		/* Control reg BAR offset */
	struct engine_sgdma_regs *sgdma_regs;	/* SGDAM reg BAR offset */
	u32 bypass_offset;			/* Bypass mode BAR offset */

	/* Engine state, configuration and flags */
	enum shutdown_state shutdown;	/* engine shutdown mode */
	enum dma_data_direction dir;
	int device_open;	/* flag if engine node open, ST mode only */
	int running;		/* flag if the driver started engine */
	int non_incr_addr;	/* flag if non-incremental addressing used */
	int streaming;
	int addr_align;		/* source/dest alignment in bytes */
	int len_granularity;	/* transfer length multiple */
	int addr_bits;		/* HW datapath address width */
	int channel;		/* engine indices */
	int max_extra_adj;	/* descriptor prefetch capability */
	int desc_dequeued;	/* num descriptors of completed transfers */
	u32 status;		/* last known status of device */
	u32 interrupt_enable_mask_value;/* only used for MSIX mode to store per-engine interrupt mask value */

	/* Transfer list management */
	struct list_head transfer_list;	/* queue of transfers */

	/* Members applicable to AXI-ST C2H (cyclic) transfers */
	struct xdma_result *cyclic_result;
	dma_addr_t cyclic_result_bus;	/* bus addr for transfer */
	struct xdma_request_cb *cyclic_req;
	struct sg_table cyclic_sgt;
	u8 eop_found; /* used only for cyclic(rx:c2h) */

	int rx_tail;	/* follows the HW */
	int rx_head;	/* where the SW reads from */
	int rx_overrun;	/* flag if overrun occured */

	/* for copy from cyclic buffer to user buffer */
	unsigned int user_buffer_index;

	/* Members associated with polled mode support */
	u8 *poll_mode_addr_virt;	/* virt addr for descriptor writeback */
	dma_addr_t poll_mode_bus;	/* bus addr for descriptor writeback */

	/* Members associated with interrupt mode support */
	wait_queue_head_t shutdown_wq;	/* wait queue for shutdown sync */
	spinlock_t lock;		/* protects concurrent access */
	int prev_cpu;			/* remember CPU# of (last) locker */
	int msix_irq_line;		/* MSI-X vector for this engine */
	u32 irq_bitmask;		/* IRQ bit mask for this engine */
	struct work_struct work;	/* Work queue for interrupt handling */

	spinlock_t desc_lock;		/* protects concurrent access */
	dma_addr_t desc_bus;
	struct xdma_desc *desc;

	/* for performance test support */
	struct xdma_performance_ioctl *xdma_perf;	/* perf test control */
	wait_queue_head_t xdma_perf_wq;	/* Perf test sync */
};

u32 engine_status_read(struct xdma_engine *engine, bool clear, bool dump);
void engine_transfer_dequeue(struct xdma_engine *engine);
void engine_service_shutdown(struct xdma_engine *engine);
int engine_init_regs(struct xdma_engine *engine);

int engine_init(struct xdma_engine *engine, struct xdma_dev *xdev,
		int offset, enum dma_data_direction dir, int channel);
struct xdma_transfer *engine_start(struct xdma_engine *engine);
int engine_addrmode_set(struct xdma_engine *engine, unsigned long arg);

void engine_stop(struct xdma_engine *engine);
void engine_destroy(struct xdma_dev *xdev, struct xdma_engine *engine);
int engine_service_poll(struct xdma_engine *engine, u32 expected_desc_count);

#endif  // XDMA_ENGINE_H
