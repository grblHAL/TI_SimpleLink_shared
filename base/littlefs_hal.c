/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A little flash file system for the Raspberry Pico
 *
 */

// Code lifted from https://github.com/lurk101/pico-littlefs

#include "driver.h"
#include "littlefs_hal.h"

#define FS_SIZE           (512 * 1024)
#define FLASH_SIZE        0x00100000
#define FLASH_PAGE_SIZE   128
#define FLASH_SECTOR_SIZE 16384

#if (FS_SIZE & (FLASH_SECTOR_SIZE - 1))
#error "Illegal littlefs file system size!"
#endif

typedef struct {
    const char *baseaddr;
#if LFS_THREADSAFE
    recursive_mutex_t fs_mtx;
#endif
} ti_lfs_context_t;

static int ti_hal_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, void* buffer, lfs_size_t size)
{
    assert(block < c->block_count);
    assert(offset + size <= c->block_size);
    // read flash via XIP mapped space
    memcpy(buffer, ((ti_lfs_context_t *)c->context)->baseaddr + (block * c->block_size) + offset, size);

    return LFS_ERR_OK;
}

static int ti_hal_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t offset, const void* buffer, lfs_size_t size)
{
    FlashProgram((uint32_t *)buffer, (uint32_t)(((ti_lfs_context_t *)c->context)->baseaddr + block * c->block_size + offset), size);

    return LFS_ERR_OK;
}

static int ti_hal_erase(const struct lfs_config *c, lfs_block_t block)
{
    assert(block < c->block_count);

    FlashErase((uint32_t)(((ti_lfs_context_t *)c->context)->baseaddr + block * c->block_size));

    return LFS_ERR_OK;
}

static int ti_hal_sync (const struct lfs_config *c)
{
    (void)c;

    return LFS_ERR_OK;
}

#if LFS_THREADSAFE

static recursive_mutex_t fs_mtx;

static int ti_lock(const struct lfs_config *c)
{
    recursive_mutex_enter_blocking(&((ti_lfs_context_t *)c->context)->fs_mtx);

    return LFS_ERR_OK;
}

static int ti_unlock(const struct lfs_config *c)
{
    recursive_mutex_exit(&((ti_lfs_context_t *)c->context)->fs_mtx);

    return LFS_ERR_OK;
}

#endif

struct lfs_config *ti_littlefs_hal (void)
{
    static ti_lfs_context_t ctx = {
        .baseaddr = (const char *)(FLASH_SIZE - FS_SIZE),
     };
    
    // configuration of the filesystem is provided by this struct
    // for Pico: prog size = 256, block size = 4096, so cache is 8K
    // minimum cache = block size, must be multiple
    static struct lfs_config ti_cfg = {
        .context = &ctx,
        // block device operations
        .read = ti_hal_read,
        .prog = ti_hal_prog,
        .erase = ti_hal_erase,
        .sync = ti_hal_sync,
    #if LFS_THREADSAFE
        .lock = ti_lock,
        .unlock = ti_unlock,
    #endif
        // block device configuration
        .read_size = 1,
        .prog_size = FLASH_PAGE_SIZE,
        .block_size = FLASH_SECTOR_SIZE,
        .block_count = FS_SIZE / FLASH_SECTOR_SIZE,
        .cache_size = FLASH_SECTOR_SIZE / 4,
        .lookahead_size = 32,
        .block_cycles = 500
    };

#if LFS_THREADSAFE
    recursive_mutex_init(&ctx.fs_mtx);
#endif

    return &ti_cfg;
}
