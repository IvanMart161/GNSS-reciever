.PHONY: run gen clean

all: run

gen:
	@cmake -B build
	@cmake --build build --target gen

run:
	@cmake -B build
	@cmake --build build --target run

clean:
	@rm -rf build bin/*.bin
