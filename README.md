# Simulator #

Simulator is a open-source executable made with python language to simulate the field of **[UnBall team](http://equipeunball.com.br/)**, in order to make it possible test the vision and too test the strategy.

-> Simulator was create using the [Box-2D](https://github.com/pybox2d/pybox2d), [pygame](https://www.pygame.org/news) and [tkinter](https://docs.python.org/3/library/tk.html) libraries.

## Installation ##
Into the folder that contains the simulator

* Make the script install_dependences executable :

>Ubuntu:
    
```bash
chmod +x install_dependences.sh
```

* Run the script install_dependences

>Ubuntu:

```bash 
sudo ./install_dependences.sh
```

* Into the folder catkin_ws_unball run the script:

> Ubuntu: 

```bash 
./run_strategy_and_pythonsimulator.sh
```

## References ##
Acess the bellow links in order to understanding how Python_Simulation works:

> Field:

- https://github.com/pybox2d/pybox2d/wiki/manual
- http://www.iforce2d.net/b2dtut/constant-speed
- http://www.iforce2d.net/b2dtut/top-down-car
- https://github.com/pybox2d/pybox2d/blob/master/examples/top_down_car.py
 
> Star Menu:

- https://github.com/ppizarror/pygame-menu

After install Pybox2D don't forget to use 
```bash 
python examples.top_down_car --backend=pygame
```
into the folder Pybox2D to see "top_dow_car.py" working.
Python_Simulation was based on script top_down_car.

## License ##

