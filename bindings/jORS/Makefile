#Makefile for jORS
# (c) LAAS-CNRS 2010

PREFIX ?= /usr/local

BUILD_DIR = build
SRC_DIR = src
DOC_DIR = doc

BASE_PACKAGE = laas.openrobots.ontology

JYARP_LIB = $(PREFIX)/java/libjyarp/lib

CLASSPATH = $(JYARP_LIB)/libjyarp.jar

JAVA?= java
JAVAC?= javac
JAVADOC?= javadoc
JAR?= jar

CLEAN?= rm -rf
INSTALL?= /usr/bin/install
CP?= cp

########## Variables for Javadoc documentation ##########
WINDOWTITLE = 'jORS, a Java binding for OpenRobot Simulator'
HEADER = '<b>jORS, a Java binding for OpenRobot Simulator</b>'
BOTTOM = '<font size="-1">jORS is a part of the <a href="https://softs.laas.fr/openrobots/wiki/">OpenRobots</a> framework.<br/><br><a href="mailto:openrobots@laas.fr">openrobots@laas.fr</a><br/>LAAS-CNRS 2010</font>'
#GROUPCORE = "Core Packages" "$(BASE_PACKAGE)*"
#GROUPBACKEND  = "Ontology Backend Packages" "$(BASE_PACKAGE).backends*"
#GROUPSERVER  = "Connectors Packages" "$(BASE_PACKAGE).connectors*"
#GROUPTESTS = "Tests Packages" "$(BASE_PACKAGE).tests*"
##########################################################

all : jors doc

jors: oro-build
	/bin/echo -e "Class-Path: \n `echo $(CLASSPATH) | sed 's/:/ \n /g'`" > MANIFEST.MF
	$(JAR) cfm jors.jar MANIFEST.MF -C $(BUILD_DIR) .
	$(CLEAN) MANIFEST.MF

oro-build :
	$(INSTALL) -d $(BUILD_DIR)
	$(JAVAC) -classpath $(CLASSPATH) -d $(BUILD_DIR) `find -name "*.java"`

install: oro-server
	$(INSTALL) -d ${PREFIX}/java/jors/lib
	$(INSTALL) jors.jar ${PREFIX}/java/jors/lib

distclean: clean doc-clean
	$(CLEAN) jors.jar

clean :
	$(CLEAN) $(BUILD_DIR)

doc:
	$(JAVADOC) -sourcepath $(SRC_DIR) \
	-overview $(SRC_DIR)/overview/overview.html \
	-classpath $(CLASSPATH) \
	-d $(DOC_DIR) \
	-use \
	-windowtitle $(WINDOWTITLE) \
	-doctitle $(HEADER) \
	-header $(HEADER) \
	-bottom $(BOTTOM) \
	-J-Xmx180m \
	-stylesheetfile $(SRC_DIR)/javadoc.css \
	-subpackages $(BASE_PACKAGE) \
	-encoding utf8

install-doc: doc
	cd doc && ${CP} -r . ${PREFIX}/share/doc/jors

doc-clean:
	$(CLEAN) $(DOC_DIR)

