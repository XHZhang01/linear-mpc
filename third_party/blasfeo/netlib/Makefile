###################################################################################################
#                                                                                                 #
# This file is part of BLASFEO.                                                                   #
#                                                                                                 #
# BLASFEO -- BLAS for embedded optimization.                                                      #
# Copyright (C) 2019 by Gianluca Frison.                                                          #
# Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              #
# All rights reserved.                                                                            #
#                                                                                                 #
# The 2-Clause BSD License                                                                        #
#                                                                                                 #
# Redistribution and use in source and binary forms, with or without                              #
# modification, are permitted provided that the following conditions are met:                     #
#                                                                                                 #
# 1. Redistributions of source code must retain the above copyright notice, this                  #
#    list of conditions and the following disclaimer.                                             #
# 2. Redistributions in binary form must reproduce the above copyright notice,                    #
#    this list of conditions and the following disclaimer in the documentation                    #
#    and/or other materials provided with the distribution.                                       #
#                                                                                                 #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 #
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   #
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          #
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 #
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  #
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     #
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      #
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   #
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    #
#                                                                                                 #
# Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             #
#                                                                                                 #
###################################################################################################

include ../Makefile.rule
include Makefile.netlib_cblas

BLAS_NETLIB = blas-3.8.0
BLAS_DIR = blas

CBLAS_NETLIB = cblas
CBLAS_DIR = cblas

LAPACK_NETLIB = lapack-3.8.0
LAPACK_DIR = lapack


obj_blas: | $(BLAS_DIR)
	@echo "compiling BLAS ..."
	cp $(BLAS_DIR)/make.inc.bkp $(BLAS_DIR)/make.inc
	@echo "" >> $(BLAS_DIR)/make.inc
	@echo "#" >> $(BLAS_DIR)/make.inc
	@echo "# BLASFEO's custom flags" >> $(BLAS_DIR)/make.inc
	@echo "#" >> $(BLAS_DIR)/make.inc
	@echo "OPTS = $(CFLAGS) -O3" >> $(BLAS_DIR)/make.inc
	(cd $(BLAS_DIR); $(MAKE) all)


$(BLAS_DIR):
	@echo "downloading BLAS ..."
	wget www.netlib.org/blas/$(BLAS_NETLIB).tgz -O $(BLAS_NETLIB).tgz
	mkdir -p $(BLAS_DIR)
	tar -xzf $(BLAS_NETLIB).tgz -C $(BLAS_DIR) --strip-components 1
	cp $(BLAS_DIR)/make.inc $(BLAS_DIR)/make.inc.bkp


obj_cblas: | $(CBLAS_DIR)
	@echo "compiling CBLAS ..."
	cp $(CBLAS_DIR)/Makefile.in.bkp $(CBLAS_DIR)/Makefile.in
	@echo "" >> $(CBLAS_DIR)/Makefile.in
	@echo "#" >> $(CBLAS_DIR)/Makefile.in
	@echo "# BLASFEO's custom flags" >> $(CBLAS_DIR)/Makefile.in
	@echo "#" >> $(CBLAS_DIR)/Makefile.in
	@echo "CC = $(CC)" >> $(CBLAS_DIR)/Makefile.in
	@echo "FC = $(CC:gcc=gfortran)" >> $(CBLAS_DIR)/Makefile.in
	@echo "CFLAGS = $(CFLAGS) -DADD_" >> $(CBLAS_DIR)/Makefile.in
	@echo "FFLAGS = $(CFLAGS)" >> $(CBLAS_DIR)/Makefile.in
	@echo "BLLIB = $(BLAS_DIR)/blas_LINUX.a" >> $(CBLAS_DIR)/Makefile.in
	@echo "CFLAGS += $(BLASFEO_CBLAS_DEF)" >> $(CBLAS_DIR)/Makefile.in
	(cd $(CBLAS_DIR); $(MAKE) alllib)


$(CBLAS_DIR):
	@echo "downloading CBLAS ..."
	wget www.netlib.org/blas/blast-forum/$(CBLAS_NETLIB).tgz -O $(CBLAS_NETLIB).tgz
	mkdir -p $(CBLAS_DIR)
	tar -xzf $(CBLAS_NETLIB).tgz -C $(CBLAS_DIR) --strip-components 1
	cp $(CBLAS_DIR)/Makefile.in $(CBLAS_DIR)/Makefile.in.bkp


obj_lapack: | $(LAPACK_DIR)
	@echo "compiling LAPACK ..."
	cp $(LAPACK_DIR)/make.inc.example $(LAPACK_DIR)/make.inc
	@echo "" >> $(LAPACK_DIR)/make.inc
	@echo "#" >> $(LAPACK_DIR)/make.inc
	@echo "# BLASFEO's custom flags" >> $(LAPACK_DIR)/make.inc
	@echo "#" >> $(LAPACK_DIR)/make.inc
	@echo "OPTS = $(CFLAGS) -O2 -frecursive" >> $(LAPACK_DIR)/make.inc
	@echo "DRVOPTS = $(OPTS)" >> $(LAPACK_DIR)/make.inc
	@echo "NOOPT = $(CFLAGS) -O0 -frecursive" >> $(LAPACK_DIR)/make.inc
	(cd $(LAPACK_DIR); $(MAKE) lib)


$(LAPACK_DIR):
	@echo "downloading LAPACK ..."
	wget www.netlib.org/lapack/$(LAPACK_NETLIB).tar.gz -O $(LAPACK_NETLIB).tar.gz
	mkdir -p $(LAPACK_DIR)
	tar -xzf $(LAPACK_NETLIB).tar.gz -C $(LAPACK_DIR) --strip-components 1


obj_lapacke: obj_lapack
	@echo "compiling LAPACKE ..."
	@echo "CFLAGS = $(CFLAGS)" >> $(LAPACK_DIR)/make.inc
	# XXX hack to fix a bug in lapacke in lapack-3.8.0
	sed -i 's/OBJ_B = \\/OBJ_B = lapacke_dsytrf_aa_work.o lapacke_ssytrf_aa_work.o \\/g' $(LAPACK_DIR)/LAPACKE/src/Makefile
	# XXX hack till here
	(cd $(LAPACK_DIR); $(MAKE) lapackelib)


clean:
ifneq ($(wildcard $(BLAS_DIR)), )
	make -C $(BLAS_DIR) clean
endif
#	if [ -d "./$(BLAS_DIR)" ]; then \
#		make -C $(BLAS_DIR) clean; \
#	fi
ifneq ($(wildcard $(CBLAS_DIR)), )
	make -C $(CBLAS_DIR) cleanall
endif
ifneq ($(wildcard $(LAPACK_DIR)), )
	make -C $(LAPACK_DIR) clean
endif


deep_clean:
	rm -rf $(BLAS_DIR)
	rm -rf $(BLAS_NETLIB).tgz
	rm -rf $(CBLAS_DIR)
	rm -rf $(CBLAS_NETLIB).tgz
	rm -rf $(LAPACK_DIR)
	rm -rf $(LAPACK_NETLIB).tar.gz


.PHONY: obj_blas obj_cblas obj_lapack clean deep_clean obj_lapacke
