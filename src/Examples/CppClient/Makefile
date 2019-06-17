CURDIR=$(PWD)
CARLADIR=$(CURDIR)/../../../
BINDIR=$(CURDIR)/bin
INSTALLDIR=$(CARLADIR)


define log
	@echo "\033[1;35m$(1)\033[0m"
endef

default: build

clean:
	@rm -rf $(BUILDDIR) $(INSTALLDIR)
	@rm -f ToolChain.cmake

run: build
	$(call log,Running C++ Client...)
	@$(BINDIR)/cpp_client $(ARGS)

build: $(BINDIR)/cpp_client

$(BINDIR)/cpp_client:
	$(call log,Compiling C++ Client...)
	@mkdir -p $(BINDIR)
	@$(CXX) $(CXXFLAGS) -I$(INSTALLDIR)/include -L$(INSTALLDIR)/lib \
		-o $(BINDIR)/cpp_client main.cpp \
		-Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic \
		-lpng -ltiff -ljpeg
