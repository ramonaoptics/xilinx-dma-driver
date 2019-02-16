#ifndef LIBXDMA_DEBUG_H
#define LIBXDMA_DEBUG_H
#include "libxdma.h"

void sgt_dump(struct sg_table *sgt);
void xdma_request_cb_dump(struct xdma_request_cb *req);
void dump_desc(struct xdma_desc *desc_virt);
void transfer_dump(struct xdma_transfer *transfer);

#endif  // LIBXDMA_DEBUG_H
