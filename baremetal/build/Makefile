%::
	$(MAKE) -C Debug $@
	$(MAKE) -C Release $@

all:
	$(MAKE) -C Debug
	$(MAKE) -C Release

standalone.dtb: standalone.dts
	dtc -I dts -O dtb -o standalone.dtb ../standalone.dts

standalone: standalone.dtb
	SAGE_STANDALONE=1 $(MAKE) -C Debug
	SAGE_STANDALONE=1 $(MAKE) -C Release
