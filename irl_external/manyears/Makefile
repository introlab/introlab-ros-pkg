all: installed

SVN_DIR = build/manyears-svn
SVN_URL = http://svn.code.sf.net/p/manyears/code/trunk/manyears-C
include $(shell rospack find mk)/svn_checkout.mk

installed: wiped $(SVN_DIR)
	cd $(SVN_DIR) && cmake -DCMAKE_INSTALL_PREFIX=${CURDIR} -DMANYEARS_GUI_DISABLED:bool=TRUE .
	cd $(SVN_DIR) && make install
	touch installed

clean: wipe

wiped: Makefile
	make wipe
	touch wiped

wipe:
	rm -rf build installed wiped bin doc include lib rtaudio example samples
