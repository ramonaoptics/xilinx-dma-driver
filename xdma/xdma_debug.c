#include "xdma_debug.h"

void sgt_dump(struct sg_table *sgt)
{
	int i;
	struct scatterlist *sg = sgt->sgl;

	pr_info("sgt 0x%p, sgl 0x%p, nents %u/%u.\n",
		sgt, sgt->sgl, sgt->nents, sgt->orig_nents);

	for (i = 0; i < sgt->orig_nents; i++, sg = sg_next(sg))
		pr_info("%d, 0x%p, pg 0x%p,%u+%u, dma 0x%llx,%u.\n",
			i, sg, sg_page(sg), sg->offset, sg->length,
			sg_dma_address(sg), sg_dma_len(sg));
}

void xdma_request_cb_dump(struct xdma_request_cb *req)
{
	int i;

	pr_info("request 0x%p, total %u, ep 0x%llx, sw_desc %u, sgt 0x%p.\n",
		req, req->total_len, req->ep_addr, req->sw_desc_cnt, req->sgt);
	sgt_dump(req->sgt);
	for (i = 0; i < req->sw_desc_cnt; i++)
		pr_info("%d/%u, 0x%llx, %u.\n",
			i, req->sw_desc_cnt, req->sdesc[i].addr,
			req->sdesc[i].len);
}

void dump_desc(struct xdma_desc *desc_virt)
{
	int j;
	u32 *p = (u32 *)desc_virt;
	static char * const field_name[] = {
		"magic|extra_adjacent|control", "bytes", "src_addr_lo",
		"src_addr_hi", "dst_addr_lo", "dst_addr_hi", "next_addr",
		"next_addr_pad"};
	char *dummy;

	/* remove warning about unused variable when debug printing is off */
	dummy = field_name[0];

	for (j = 0; j < 8; j += 1) {
		pr_info("0x%08lx/0x%02lx: 0x%08x 0x%08x %s\n",
			 (uintptr_t)p, (uintptr_t)p & 15, (int)*p,
			 le32_to_cpu(*p), field_name[j]);
		p++;
	}
	pr_info("\n");
}

void transfer_dump(struct xdma_transfer *transfer)
{
	int i;
	struct xdma_desc *desc_virt = transfer->desc_virt;

	pr_info("xfer 0x%p, state 0x%x, f 0x%x, dir %d, len %u, last %d.\n",
		transfer, transfer->state, transfer->flags, transfer->dir,
		transfer->len, transfer->last_in_request);

	pr_info("transfer 0x%p, desc %d, bus 0x%llx, adj %d.\n",
		transfer, transfer->desc_num, (u64)transfer->desc_bus,
		transfer->desc_adjacent);
	for (i = 0; i < transfer->desc_num; i += 1)
		dump_desc(desc_virt + i);
}
