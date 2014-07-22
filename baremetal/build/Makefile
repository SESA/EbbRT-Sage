%::
	$(MAKE) -C Debug $@
	$(MAKE) -C Release $@

all:
	$(MAKE) -C Debug
	$(MAKE) -C Release

standalone:
	SAGE_STANDALONE=1 $(MAKE) -C Debug
	SAGE_STANDALONE=1 $(MAKE) -C Release
