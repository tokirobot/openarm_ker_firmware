# Host-side verification of the RS-485 wire protocol in ../KerWireProto.h.
#
#   make -C common/test run
#
# Exhaustive where it can be: every codeword, every 1-bit error, every 2-bit
# error, every burst up to 6 bits. The CRC's detection guarantees are the reason
# jump detection can stop treating data corruption as a mechanical fault, so
# they are worth checking rather than assuming.
CFLAGS ?= -O2 -Wall -Wextra -I..

run: test_wire_proto
	./test_wire_proto

test_wire_proto: test_wire_proto.c ../KerWireProto.h
	$(CC) $(CFLAGS) -o $@ test_wire_proto.c -lm

clean:
	rm -f test_wire_proto

.PHONY: run clean
