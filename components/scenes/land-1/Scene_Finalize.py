sensor = GameLogic.getCurrentController().sensors['ESC_KEY']

#execute only when the ESC key is released (if we don't test that, 
#the code get executed two time, when pressed, and when released)
if not sensor.positive and sensor.triggered:
	print '######### FINALIZING... ########'
	
	#YarpBlender.finalize()
	GameLogic.orsConnector.finalize()
	
	cont = GameLogic.getCurrentController()
	quitActuator = cont.actuators['Quit_sim']
	cont.activate(quitActuator)
	
	print '######### EXITING SIMULATION ########'
