#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2016 Katja Fiedler, Claudia Buhl
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys, os
from PyQt4 import QtGui, QtCore
import rospy
from geometry_msgs.msg import Twist
import select 
import time
import turtlebot_easyprog.bewegen as bewegen
from thread import start_new_thread
import math


class ChangeWid( QtGui.QWidget ): #neu
  
  def __init__(self):  
    super(ChangeWid, self).__init__()

######################
######################
######################


class TurtleDance(QtGui.QWidget):
  
  trigger = QtCore.pyqtSignal() 
  
  def __init__(self):
    super(TurtleDance, self).__init__()
    self.initUI()
  
  def initUI(self):
    # ros stuff
    rospy.init_node('turtle_dance')

    #####=======================================================================
    #####== data members  ======================================================
    #####=======================================================================
    self.xAcc=0.05 # neu ff
    self.rAcc=0.5
    self.xAccCurr=0.05
    self.rAccCurr=0.5
    #####=======================================================================
    #####== create GUI elements ================================================
    #####=======================================================================
    # dummy for space 
    empty = QtGui.QLabel("\n")
    # logo in label
    pic = QtGui.QLabel()
    image_path = rospy.get_param('~image')
    #print "image: ", image_path
    pic.setPixmap(QtGui.QPixmap(image_path))
    #pic.setAlignment(QtCore.Qt.AlignCenter)
    #####=======================================================================
    # list for dancing steps
    self.programList = QtGui.QListWidget()
    self.programList.setFixedWidth(600)
    self.programList.setFixedHeight(600)
    self.programList.itemDoubleClicked.connect(self.changeAcc)
    #####=======================================================================
    self.trigger.connect(self.updateUI)
    #####=======================================================================
    # fixed width
    # sw = 80
    # spinbox: only rotation
    self.rotSpin = QtGui.QDoubleSpinBox()
    self.rotSpin.setDecimals(2)
    self.rotSpin.setRange(-math.pi,math.pi)
    self.rotSpin.setSingleStep(0.05)
    # spinbox: only move straight
    self.moveSpin = QtGui.QDoubleSpinBox()
    self.moveSpin.setDecimals(2)
    self.moveSpin.setRange(-5.0,5.0)
    self.moveSpin.setSingleStep(0.1)
    # spinbox: move for-/backwards
    self.vxSpin = QtGui.QDoubleSpinBox()
    self.vxSpin.setDecimals(2)
    self.vxSpin.setRange(-0.5,0.5)
    self.vxSpin.setSingleStep(0.05)
    # self.vxSpin.setFixedWidth(sw)
    # spinbox: turn left/right
    self.vzSpin = QtGui.QDoubleSpinBox()
    self.vzSpin.setDecimals(2)
    self.vzSpin.setRange(-3.0,3.0)
    self.vzSpin.setSingleStep(0.1)
    # spinbox: duration
    self.durSpin = QtGui.QDoubleSpinBox()
    self.durSpin.setDecimals(2)
    self.durSpin.setRange(0,5.0)
    self.durSpin.setSingleStep(0.1)
    #####=======================================================================
    # label for spinboxes 
    vxLabel = QtGui.QLabel(u"Fahren: vorwärts(+)/rückwärts(-)")
    vx1Label = QtGui.QLabel(u"Fahren: vorwärts(+)/rückwärts(-)")
    vzLabel = QtGui.QLabel(u"Drehen: links(+)/rechts(-)")
    vz1Label = QtGui.QLabel(u"Drehen: links(+)/rechts(-)")
    durLabel = QtGui.QLabel(u"Dauer der Bewegung (Sekunden):")
    #####=======================================================================
    # buttons for single step design
    bw = 80 # button height
    addAButton = QtGui.QPushButton(u'Einfügen\n danach')
    addAButton.setFixedWidth(bw)
    addAButton.clicked.connect(self.addStepAfter)
    addAButton.setToolTip(u'Schritt danach hinzufügen') #neu
    # add new step before current line
    addBButton = QtGui.QPushButton(u'Einfügen\n davor')
    addBButton.setFixedWidth(bw)
    addBButton.clicked.connect(self.addStepBefore)
    addBButton.setToolTip(u'Schritt davor hinzufügen') #neu
    # delete to selected steps
    deleteButton = QtGui.QPushButton("entfernen")
    deleteButton.setFixedWidth(bw)
    deleteButton.clicked.connect(self.deleteStep)
    deleteButton.setToolTip(u'Schritt entfernen') #neu
    #####=======================================================================
    # buttons for manipulation of dance 
    butW=80
    self.loadButton = QtGui.QPushButton("Lade\nTanz")
    self.loadButton.setFixedWidth(butW)
    self.loadButton.clicked.connect(self.loadDance)
    self.tryButton = QtGui.QPushButton("Probiere\nTanz")
    self.tryButton.setFixedWidth(butW)
    self.tryButton.clicked.connect(self.tryDance)
    self.saveButton = QtGui.QPushButton("Speichere\nTanz")
    self.saveButton.setFixedWidth(butW)
    self.saveButton.clicked.connect(self.saveDance)
    clearButton = QtGui.QPushButton(u"Lösche\nEntwurf")
    clearButton.setFixedWidth(butW)
    clearButton.clicked.connect(self.clearList)
    #####=======================================================================
    # select platform
    self.robotCombo = QtGui.QComboBox()
    self.robotCombo.addItem("Turtle-Grafik")
    self.robotCombo.addItem("Gazebo-Simulation")
    self.robotCombo.addItem("Echter Roboter")
    # label for platform
    platfLab = QtGui.QLabel(u"Wähle den tanzenden Roboter aus:")
    #####=======================================================================
    #####== build layout =======================================================
    #####=======================================================================
    # tab: dance control + dance display
    hboxLeft = QtGui.QVBoxLayout()
    hboxLeft.addWidget(vx1Label)
    hboxLeft.addWidget(self.vxSpin)
    hboxLeft.addWidget(vz1Label)
    hboxLeft.addWidget(self.vzSpin)
    hboxLeft.addWidget(durLabel)
    hboxLeft.addWidget(self.durSpin)
    hboxWidget = QtGui.QWidget()
    hboxWidget.setLayout(hboxLeft)
    #####=======================================================================
    # tab: exact move
    hboxMove = QtGui.QVBoxLayout()
    hboxMove.addWidget(vxLabel)
    hboxMove.addWidget(self.moveSpin)
    hboxMoveWidget = QtGui.QWidget()
    hboxMoveWidget.setLayout(hboxMove)
    #####=======================================================================
    # tab: exact rotate
    hboxRot = QtGui.QVBoxLayout()
    hboxRot.addWidget(vzLabel)
    hboxRot.addWidget(self.rotSpin)
    hboxRotWidget = QtGui.QWidget()
    hboxRotWidget.setLayout(hboxRot)
    #####=======================================================================
    hboxMid = QtGui.QGridLayout()
    hboxMid.addWidget(self.tryButton,0,2)
    hboxMid.addWidget(self.loadButton,1,2)
    hboxMid.addWidget(self.saveButton,2,2)
    hboxMid.addWidget(clearButton,3,2)
    #####=======================================================================
    hboxMid.addWidget(addAButton,1,1)
    hboxMid.addWidget(addBButton,2,1)
    hboxMid.addWidget(deleteButton,3,1)
    #####=======================================================================
    tabs = QtGui.QTabWidget()
    tabs.addTab(hboxWidget, "einfach")
    tabs.addTab(hboxMoveWidget, "exakt fahren")
    tabs.addTab(hboxRotWidget, "exakt rotieren")
    tabs.currentChanged.connect(self.tabChanged)
    mid = QtGui.QHBoxLayout()
    mid.addWidget(tabs)
    mid.addLayout(hboxMid)
    #####=======================================================================
    vboxLeft = QtGui.QVBoxLayout()
    vboxLeft.addWidget(pic, QtCore.Qt.AlignCenter)
    vboxLeft.addStretch(1)
    vboxLeft.addWidget(empty)
    vboxLeft.addStretch(1)
    vboxLeft.addWidget(platfLab,0)
    vboxLeft.addWidget(self.robotCombo,0)
    vboxLeft.addStretch(1)
    vboxLeft.addLayout(mid)
    #####=======================================================================
    hboxGui = QtGui.QHBoxLayout()
    hboxGui.addLayout(vboxLeft)
    hboxGui.addWidget(self.programList)
    #####=======================================================================
    self.selectedItem = 0
    self.selectedTab = 0

    self.setLayout(hboxGui)    

    self.setGeometry(50, 50, 1000, 800)
    self.setWindowTitle('TurtleDance')    
    self.setFixedSize( self.sizeHint() )
    self.layout().setSizeConstraint( QtGui.QLayout.SetFixedSize )
    self.show()

    print u"\nViel Spaß!\n"

  ##############################################################################
  
  # remove all items from dance list
  def clearList( self ):
    self.programList.clear()

  ##############################################################################
 
  # create step from current spinbox settings and
  # add before currently selected item
  # (or at the end, if nothing selected)
  def addStepBefore( self ):
    # create content
    if self.selectedTab == 0:
      item = QtGui.QListWidgetItem("= vor %.2f = dreh %.2f = dauer %.2f = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
      self.vxSpin.value(), self.vzSpin.value(), self.durSpin.value(), self.xAcc, self.rAcc )) # neu
    elif self.selectedTab == 1:
      item = QtGui.QListWidgetItem("= exakt vor %.2f " %( self.moveSpin.value())) # neu
    else:
      item = QtGui.QListWidgetItem("= exakt dreh %.2f " %( self.rotSpin.value())) # neu
    # get selected index
    ind = self.programList.selectedIndexes()
    # if somethings is selected 
    if len( ind ) > 0:
      # add behind current position
      self.programList.insertItem(ind[0].row(), item )
    else:
      # simply add
      self.programList.addItem(item)

  ##############################################################################
  
  # create step from current spinbox settings and
  # add after currently selected item
  # (or at the end, if nothing selected)
  def addStepAfter( self ):
    # create content
    if self.selectedTab == 0:
      item = QtGui.QListWidgetItem("= vor %.2f = dreh %.2f = dauer %.2f = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
      self.vxSpin.value(), self.vzSpin.value(), self.durSpin.value(), self.xAcc, self.rAcc )) # neu
    elif self.selectedTab == 1:
      item = QtGui.QListWidgetItem("= exakt vor %.2f " %( self.moveSpin.value())) # neu
    else:
      item = QtGui.QListWidgetItem("= exakt dreh %.2f " %( self.rotSpin.value())) # neu
    # get selected index
    ind = self.programList.selectedIndexes()
    # if somethings is selected 
    if len( ind ) > 0:
      # add behind current position
      self.programList.insertItem(ind[0].row()+1, item )
    else:
      # simply add
      self.programList.addItem(item)

  ##############################################################################
 
  # if item of dance list selected, use its values for spinbox settings
  def loadStep( self ):
    # get content of current description in list
    ind = self.programList.selectedIndexes()
    if len( ind )>0:
      # split
      s = str(ind[0].data().toString())
      l=s.split()
      # get values
      self.vxSpin.setValue( float( l[2] ) )
      self.vzSpin.setValue( float( l[5] ) )
      self.durSpin.setValue( float( l[8] ) )

  ##############################################################################
  
  # if item in dance list selected, 
  # substitute it by current dance settings from spinboxes
  def changeStep( self ):
    # prepare step description
    item = QtCore.QString("= vor %.2f = dreh %.2f = dauer %.2f = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
    self.vxSpin.value(), self.vzSpin.value(), self.durSpin.value(), self.xAccCurr, self.rAccCurr )) # neu# TODO
    # get index of current list item (we can only select one)
    it = self.programList.selectedItems()
    # write text to current list item
    if len( it ) > 0:
      it[0].setText( item ) 

  ##############################################################################
  
  # delete currently selected step, if any
  def deleteStep( self ):
    # get index of current list item (we can only select one)
    ind = self.programList.selectedIndexes()
    # remove the current list item
    if len( ind ) > 0:
      self.programList.takeItem(ind[0].row()) 
  
  ##############################################################################
 
  # load list of steps from desired file
  def loadDance( self ):
    # get filename and open

    dances_path = rospy.get_param('~dances_path')
    fname = QtGui.QFileDialog.getOpenFileName(self, 'Tanz laden:', dances_path + "/", "*.dc")
    if len(fname)>0:
      file = open(fname,"r")
      # read data and tidy up
      lines = file.readlines()
      file.close()
      # empty list of steps
      self.programList.clear()
      # put data into liste of steps
      for li in lines:
        # get elements
        l=li.split()
        # create item
        item = QtGui.QListWidgetItem("= vor %s = dreh %s = dauer %s = ( vorBeschl %s , drehBeschl %s )" \
        %( l[0], l[1], l[2], l[3], l[4] ) ) # neu
        # add item
        self.programList.addItem(item)
    print "Tanz geladen.\n"

  ##############################################################################

  # save list of steps in desired file
  def saveDance( self ):
    dances_path = rospy.get_param('~dances_path')
    fname = QtGui.QFileDialog.getSaveFileName(self, 'Speichern unter', dances_path + "/", "*.dc") #'/home')
    if len(fname)>0:
      if fname[len(fname)-3:len(fname)] != ".dc":
        fname = fname + ".dc"
      if self.programList.count()>0:
        file = open(fname,"w")
        i = 0
        while i < self.programList.count():
          item = self.programList.item(i)
          i += 1
          # write velocities to file
          s=str(item.text())
          l=s.split()
          print >> file, l[2], l[5], l[8], l[12], l[15]
        file.close()
        print "Tanz gepeichert.\n"
      else:
        print "Keine Schritte vorhanden!\n"

  ##############################################################################

  def danceInThread( self, itemlist, topic):
    i = 0
    while i < len(itemlist):
      self.selectedItem = i
      item = self.programList.item(i)
      self.trigger.emit()
      l = str(item.text()).split()
      if len(l) == 17: # einfache Fahrkommandos
          target_speed = float(l[2]) # speed * x
          target_turn  = float(l[5]) # turn * th
          pub_dur = float(l[8])
          acc_speed = float(l[12])
          acc_turn  = float(l[15])
          rospy.loginfo ("Zielgeschwindigkeiten: vor = %.2f; dreh = %.2f; Dauer = %.2fs; ", target_speed, target_turn, pub_dur);
          bewegen.kurvegeschmeidig(target_speed, acc_speed, target_turn, acc_turn, pub_dur, topic)
      elif "exakt vor" in  str(item.text()): # exakt fahren
          target_length = float(l[3])
          rospy.loginfo ("Streckenlänge: %.2f", target_length);
          bewegen.fahreninmeter(target_length)
      else: # exakt drehen
          target_turn = float(l[3])
          rospy.loginfo ("Drehwinkel: %.2f", target_turn);
          bewegen.dreheninrad(target_turn)
      i = i+1
    
    time.sleep(0.5)
    print "Tanz ist fertig!\n"
    self.tryButton.setDisabled (False)

  ##############################################################################

  # execute dance steps from file on current robot
  def tryDance( self ):
    # type of robot
    print u"Gewählte Plattform:", str(self.robotCombo.currentText())
    pltf=str(self.robotCombo.currentText()) 
    if pltf == "Turtle-Grafik":
      topic = "/turtle1/cmd_vel"
    elif pltf == "Gazebo-Simulation":
      topic = "/mobile_base/commands/velocity"
    else:
      topic = "/mobile_base/commands/velocity"
    print "   topic name: ", topic

    # Buttons deaktivieren
    self.tryButton.setDisabled (True)

    itemlist = []
    i = 0
    while i < self.programList.count():
      item = self.programList.item(i)
      itemlist.append(item)
      i = i+1
    
    start_new_thread(self.danceInThread, (itemlist, topic,))
  
  ##############################################################################

  def updateUI ( self ):
    item = self.programList.item(self.selectedItem)
    item.setSelected(True)
    self.programList.setFocus()
    self.repaint()

  ##############################################################################

  # apply change in acceleration 
  def applyAcc( self ): # neu
    # read old values
    ind = self.programList.selectedIndexes()
    if len( ind ) > 0:
      # split
      s = str(ind[0].data().toString())
      l = s.split()
      # get values
      self.vxSpin.setValue( float( l[2] ) )
      self.vzSpin.setValue( float( l[5] ) )
      self.durSpin.setValue( float( l[8] ) )
    # prepare new step description
    item = QtCore.QString("= vor %.2f = dreh %.2f = dauer %.2f = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
    self.xlinSpin.value(), self.xrotSpin.value(), self.xdurSpin.value(), self.xAccSpin.value(), self.rAccSpin.value() )) 
    # get index of current list item (we can only select one)
    it = self.programList.selectedItems()
    # write text to current list item
    if len( it ) > 0:
      it[0].setText( item )
    self.cw.close()

  ##############################################################################
  # change acceleration on demand (double click on list)
  def changeAcc( self ): # neu
    # get content of current description in list
    ind = self.programList.selectedIndexes()
    # split
    s = str(ind[0].data().toString())
    l = s.split()
    if len(l) < 17:
        msg = QtGui.QMessageBox()
        msg.setIcon(QtGui.QMessageBox.Information)
        msg.setText("Diese Funktion ist noch nicht implementiert :-(")
        msg.setWindowTitle("Message")
        msg.setStandardButtons(QtGui.QMessageBox.Cancel)
        msg.exec_()
        return

    # open new widget
    self.cw = ChangeWid()
    self.cw.setGeometry(QtCore.QRect(750, 250, 200, 200))
    self.cw.setWindowTitle(u'Beschleunigung ändern')
    # spinbox: linear speed
    self.xlinSpin = QtGui.QDoubleSpinBox()
    self.xlinSpin.setDecimals(2)
    self.xlinSpin.setRange(-0.5,0.5)
    self.xlinSpin.setSingleStep(0.05)
    self.xlinSpin.setValue(float( l[2] ))
    # spinbox: rotation speed
    self.xrotSpin = QtGui.QDoubleSpinBox()
    self.xrotSpin.setDecimals(2)
    self.xrotSpin.setRange(-3.0,3.0)
    self.xrotSpin.setSingleStep(0.1)
    self.xrotSpin.setValue(float( l[5] ))
    # spinbox: duration
    self.xdurSpin = QtGui.QDoubleSpinBox()
    self.xdurSpin.setDecimals(2)
    self.xdurSpin.setRange(0.0,5.0)
    self.xdurSpin.setSingleStep(0.1)
    self.xdurSpin.setValue(float( l[8] ))
    # spinbox: movement acceleration
    self.xAccSpin = QtGui.QDoubleSpinBox()
    self.xAccSpin.setDecimals(2)
    self.xAccSpin.setRange(0.01,0.05)
    self.xAccSpin.setSingleStep(0.01)
    self.xAccSpin.setValue(float( l[12] ))
    # spinbox: turning acceleration
    self.rAccSpin = QtGui.QDoubleSpinBox()
    self.rAccSpin.setDecimals(2)
    self.rAccSpin.setRange(0.01,0.7)
    self.rAccSpin.setSingleStep(0.01)
    self.rAccSpin.setValue(float( l[15] ))
    # labels
    xAccLabel = QtGui.QLabel(u"Beschleunigung vorwärts/rückwärts")
    rAccLabel = QtGui.QLabel(u"Beschleunigung drehen")
    xlinLabel = QtGui.QLabel(u"Fahren: vorwärts(+)/rückwärts(-)")
    rrotLabel = QtGui.QLabel(u"Drehen: links(+)/rechts(-)")
    xdurLabel = QtGui.QLabel(u"Dauer der Bewegung (Sekunden):")

    cwLayout = QtGui.QVBoxLayout()
    cwLayout.addWidget(xlinLabel)
    cwLayout.addWidget(self.xlinSpin)
    cwLayout.addWidget(rrotLabel)
    cwLayout.addWidget(self.xrotSpin)
    cwLayout.addWidget(xdurLabel)
    cwLayout.addWidget(self.xdurSpin)
    cwLayout.addWidget(xAccLabel)
    cwLayout.addWidget(self.xAccSpin)
    cwLayout.addWidget(rAccLabel)
    cwLayout.addWidget(self.rAccSpin)
    
    btLayout = QtGui.QHBoxLayout()
    
    abButton = QtGui.QPushButton("Abbrechen")
    abButton.clicked.connect(self.cw.close)
    abButton.setToolTip(u'Bisherige Beschleunigung beibehalten') #neu
    okButton = QtGui.QPushButton(u"Übernehmen")
    okButton.clicked.connect(self.applyAcc)
    okButton.setToolTip(u'Setze neue Beschleunigung') #neu
   
    btLayout.addWidget(abButton)
    btLayout.addWidget(okButton)

    cwLayout.addLayout(btLayout)
    self.cw.setLayout(cwLayout)
    
    
    self.cw.show()

  ##############################################################################
  
  def tabChanged( self , index):
    self.selectedTab = index
    if index == 0:
      self.loadButton.setDisabled (False)
      self.saveButton.setDisabled (False)
      self.robotCombo.clear()
      self.robotCombo.addItem("Turtle-Grafik")
      self.robotCombo.addItem("Gazebo-Simulation")
      self.robotCombo.addItem("Echter Roboter")
    elif index == 1:
      self.loadButton.setDisabled (True)
      self.saveButton.setDisabled (True)
      self.robotCombo.clear()
      self.robotCombo.addItem("Gazebo-Simulation")
      self.robotCombo.addItem("Echter Roboter")
    else:
      self.loadButton.setDisabled (True)
      self.saveButton.setDisabled (True)
      self.robotCombo.clear()
      self.robotCombo.addItem("Gazebo-Simulation")
      self.robotCombo.addItem("Echter Roboter")
    self.repaint()
      

###########################################################

def main():

  app = QtGui.QApplication(sys.argv)
  ex = TurtleDance()
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
