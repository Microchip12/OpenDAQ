SUMMARY = "Xilinx DMA Proxy kernel module"
LICENSE = "Apache-2.0"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/Apache-2.0;md5=89aea4e17d99a7cacdbeed46a0096b10"

FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

inherit module

SRC_URI = "file://dma-proxy.c \
           file://dma-proxy.h \
           file://Makefile \
"

S = "${WORKDIR}"



