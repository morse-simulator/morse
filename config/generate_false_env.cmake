#SET (SPHINX_FALSE_PYTHON_ENV ${CMAKE_CURRENT_BINARY_DIR}/fakeenv)

FILE(WRITE ${SPHINX_FALSE_PYTHON_ENV}/hla/omt.py "HLAfloat32LE=0\ndef HLAfixedArray(x,y,z):\n    pass")
FILE(WRITE ${SPHINX_FALSE_PYTHON_ENV}/hla/rti.py "class FederateAmbassador:\n    pass")
