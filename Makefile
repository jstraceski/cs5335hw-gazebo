
all:
	(cd cmd && make)
	(cd brain && make)
	(cd straceski_joseph-hw01 && make)
	(cd plugins/car_control && make)

clean:
	(cd cmd && make clean)
	(cd brain && make clean)
	(cd straceski_joseph-hw01 && make clean)
	(cd plugins/car_control && make clean)

.PHONY: all clean
