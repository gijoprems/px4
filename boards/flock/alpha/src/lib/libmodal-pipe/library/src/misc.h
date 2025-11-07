/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 ******************************************************************************/



#ifndef PIPE_MISC_H
#define PIPE_MISC_H

/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


#include <stdio.h>  // for fprintf
#include <stdint.h>

/**
 * @brief      helper to make a new directory and all necessary parent
 *             directories. Returns success if folder(s) already exist.
 *
 *             This requires the directory string to contain a trailing '/'
 *             after the final directory. This is done to allow a full path to a
 *             file to be given and this function will only create the necessary
 *             directories.
 *
 *             For example:
 *
 *             _mkdir_recursive("/tmp/folder1/folder2/"
 *             _mkdir_recursive("/tmp/folder1/folder2/file1"
 *
 *             will BOTH create: /tmp/ /tmp/folder1/ and /tmp/folder1/folder2/
 *
 *             Neither will create a folder named "file1" as mkdir() would.
 *
 * @param[in]  dir   The directory string
 *
 * @return     0 on success, -1 on failure
 */
int _mkdir_recursive(const char* dir);


/**
 * @brief      equivalent to rm -r
 *
 * @param      path  The directory to remove
 *
 * @return     0 on success, -1 on failure
 */
int _remove_recursive(const char *path);


/**
 * wrapper for access() to check if a file exists to make code more readable
 */
int _exists(char* path);


// current clock time monotonic in nanoseconds
int64_t _time_monotonic_ns(void);


#endif // PIPE_MISC_H
