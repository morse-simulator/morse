
Media
=====

Videos
------

+------------------------------------------+------------------------------------------+
|                                          |                                          |
|         .. vimeo:: 44505701              |            .. vimeo:: 23244699           |
|            :width: 400                   |                :width: 400               |
|                                          |                                          |
|  A compilation of simulated scenarios    |  Using ROS RVIZ visualisation tool to    |
|  used in several research laboratories.  |  display the video stream of a simulated |
|                                          |  camera                                  |
+------------------------------------------+------------------------------------------+
|                                          |                                          |
|         .. vimeo:: 27862570              |            .. vimeo:: 19258005           |
|            :width: 400                   |               :width: 400                |
|                                          |                                          |
|  Some examples of integration between    | MORSE can take care of navigation if     |
|  MORSE and ROS in human-robot interaction| you do not want to manage this part.     |
|  context.                                |                                          |
+------------------------------------------+------------------------------------------+
|                                          |                                          |
|         .. vimeo:: 22246759              |            .. vimeo:: 9825826            |
|            :width: 400                   |               :width: 400                |
|                                          |                                          |
| Here, a robot builds a 2.5D terrain model|   Several robots following each other.   |
|  map from simulated stereo-cameras, and  |     Each robot streams its camera, here  |
|  uses it to navigate.                    |                with YARP.                |
+------------------------------------------+------------------------------------------+
|                                          |                                          |
|         .. vimeo:: 27862605              |            .. vimeo:: 80372226           |
|            :width: 400                   |               :width: 400                |
|                                          |                                          |
|  MORSE can be used to test human-robot   |    MORSE simulates a Velodyne            |
|  interaction scenarii, where a human     |    used to build a map of the            |
|  is controlled as in a video game.       |    environment.                          |
+------------------------------------------+------------------------------------------+

More videos are available on the `Blender for Robotics Vimeo group 
<http://vimeo.com/groups/blenderandrobotics>`_.


Screenshots
-----------

+------------------------------------------+------------------------------------------+
| .. figure:: ../media/caylus.jpg          |  .. figure:: ../media/indoors_sick.jpg   | 
|    :width: 422                           |                                          |
|                                          |     Real-time simulation of a SICK       |
|    Simulation of ground-air cooperation. |     laser range finder in an indoors     |
|                                          |     environment.                         |
+------------------------------------------+------------------------------------------+
| .. figure:: ../media/outdoor_example.jpg |  .. figure:: ../media/ocean.jpg          | 
|                                          |     :width: 422                          |
|                                          |                                          |
|    An ATRV in an outdoor scenario.       |     Cooperation between an helicopter    |
|                                          |     and a submarine for mine hunting.    |
|                                          |                                          |
+------------------------------------------+------------------------------------------+
| .. figure:: ../media/hri.jpg             |  .. figure:: ../media/morse_interface.jpg| 
|    :width: 422                           |     :width: 422                          |
|                                          |                                          |
|    Simulation of human-robot             |     The MORSE interface (crude Blender   |
|    interaction: the robot tracks the     |     :-) )                                |
|    posture of the human.                 |                                          |
+------------------------------------------+------------------------------------------+

MORSE related academic publications
===================================

- `Simulating Complex Robotic Scenarios with MORSE <http://www.openrobots.org/morse/material/media/pdf/SIMPAR_2012.pdf>`_, SIMPAR 2012::

	@inproceedings{morseSIMPAR2012,
	  author    = {Gilberto Echeverria and
				   Séverin Lemaignan and
				   Arnaud Degroote and
				   Simon Lacroix and
				   Michael Karg and
				   Pierrick Koch and
				   Charles Lesire and
				   Serge Stinckwich},
	  title     = {Simulating Complex Robotic Scenarios with MORSE},
	  booktitle = {SIMPAR},
	  year      = {2012},
	  pages     = {197-208},
	  ee        = {http://dx.doi.org/10.1007/978-3-642-34327-8_20},
	}
- `Human-robot interaction in the MORSE simulator <http://hal.archives-ouvertes.fr/docs/00/66/70/26/PDF/lemaignan2012morse.pdf>`_, HRI 2012::

	@inproceedings{lemaignan2012human,
	  title={Human-robot interaction in the MORSE simulator},
	  author={Lemaignan, S. and Echeverria, G. and Karg, M. and Mainprice, J. and Kirsch, A. and Alami, R.},
	  booktitle={Proceedings of the seventh annual ACM/IEEE international conference on Human-Robot Interaction},
	  pages={181--182},
	  year={2012},
	  organization={ACM}
	}


- `Modular Open Robots Simulation Engine: MORSE <http://www.openrobots.org/morse/material/media/pdf/paper-icra.pdf>`_, ICRA 2011::

    @InProceedings{morseICRA2011,
        author = {G. Echeverria and N. Lassabe and A. Degroote and S. Lemaignan}
        title = {Modular OpenRobots Simulation Engine: MORSE}
        booktitle = {Proceedings of the IEEE ICRA},
        year = {2011}
    }

- Presentation of MORSE at the Blender Conference 2010:
  `slides <http://www.openrobots.org/morse/material/media/pdf/BC_morse.pdf>`_ and
  `video <http://www.youtube.com/watch?v=BGDfbi28s14#t=20m20s>`_
- Presentation at the `European Robotics Forum 2011 <http://www.eurobotics-project.eu/cms/index.php?idcat=40>`_:
  `general overview of MORSE <http://www.openrobots.org/morse/material/media/pdf/ERF-MORSE-presentation.pdf>`_ and
  `MORSE-ROS implementation <http://www.openrobots.org/morse/material/media/pdf/morse_ros.pdf>`_
