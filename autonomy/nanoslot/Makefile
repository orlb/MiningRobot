DIRS=nanoboot slot_A0 slot_EE ../backend

all:
	for dir in $(DIRS) ; do \
		make -C "$$dir" $@;\
	done

clean:
	for dir in $(DIRS) ; do \
		make -C "$$dir" $@;\
	done
	- rm /tmp/data_exchange/nanoslot
