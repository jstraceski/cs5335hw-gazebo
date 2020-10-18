
<<<<<<< HEAD
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
=======

clean:
	(cd hw01 && make clean)
	(cd hw05 && make clean)
	(cd hw06 && make clean)
	(cd xingyu01 && make clean)
>>>>>>> 548f7733320bc85e48c454f57f982aef7273797a

.PHONY: clean
