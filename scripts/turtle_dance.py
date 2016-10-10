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


class ChangeWid( QtGui.QWidget ): #neu
  
  def __init__(self):  
    super(ChangeWid, self).__init__()

######################
######################
######################


class TurtleDance(QtGui.QWidget):

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
    # spinbox: move for-/backwards
    self.vxSpin = QtGui.QDoubleSpinBox()
    self.vxSpin.setDecimals(2)
    self.vxSpin.setRange(-0.5,0.5)
    self.vxSpin.setSingleStep(0.01)
    # spinbox: turn left/right
    self.vzSpin = QtGui.QDoubleSpinBox()
    self.vzSpin.setDecimals(2)
    self.vzSpin.setRange(-3.0,3.0)
    self.vzSpin.setSingleStep(0.02)
    # spinbox: duration
    self.durSpin = QtGui.QDoubleSpinBox()
    self.durSpin.setDecimals(2)
    self.durSpin.setRange(0,5.0)
    self.durSpin.setSingleStep(0.2)
    #####=======================================================================
    # label for spinboxes 
    vxLabel = QtGui.QLabel(u"Fahren: vorwärts(+)/rückwärts(-)")
    vzLabel = QtGui.QLabel(u"Drehen: links(+)/rechts(-)")
    durLabel = QtGui.QLabel(u"Dauer der Bewegung (Sekunden):")
    #####=======================================================================
    # buttons for single step design
    bh = 80 # button width
    bw = 50 # button height
    # add new step after current line
    addAButton = QtGui.QPushButton("...\n>>>")
    addAButton.setFixedWidth(bw)
    addAButton.setFixedHeight(bh)
    addAButton.clicked.connect(self.addStepAfter)
    addAButton.setToolTip(u'Danach Schritt hinzufügen') #neu
    # add new step before current line
    addBButton = QtGui.QPushButton(">>>\n...")
    addBButton.setFixedWidth(bw)
    addBButton.setFixedHeight(bh)
    addBButton.clicked.connect(self.addStepBefore)
    addBButton.setToolTip(u'Davor Schritt hinzufügen') #neu
    # change values of selected step
    redoButton = QtGui.QPushButton("<")
    redoButton.setFixedWidth(bw)
    redoButton.setFixedHeight(bh)
    redoButton.clicked.connect(self.loadStep)
    redoButton.setToolTip(u'Schritt bearbeiten') #neu
    # submit changes to selected step
    changeButton = QtGui.QPushButton(">")
    changeButton.setFixedWidth(bw)
    changeButton.setFixedHeight(bh)
    changeButton.clicked.connect(self.changeStep)
    changeButton.setToolTip(u'Änderung übernehmen') #neu
    # delete to selected steps
    deleteButton = QtGui.QPushButton("x")
    deleteButton.setFixedWidth(bw)
    deleteButton.setFixedHeight(bh)
    deleteButton.clicked.connect(self.deleteStep)
    deleteButton.setToolTip(u'Schritt entfernen') #neu
    #####=======================================================================
    # buttons for manipulation of dance 
    butW=80
    loadButton = QtGui.QPushButton("Lade\nTanz")
    loadButton.setFixedWidth(butW)
    loadButton.clicked.connect(self.loadDance)
    self.tryButton = QtGui.QPushButton("Probiere\nTanz")
    self.tryButton.setFixedWidth(butW)
    self.tryButton.clicked.connect(self.tryDance)
    saveButton = QtGui.QPushButton("Speichere\nTanz")
    saveButton.setFixedWidth(butW)
    saveButton.clicked.connect(self.saveDance)
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
    # lower part: dance control + dance display
    hboxMid = QtGui.QHBoxLayout()
    vboxMid1 = QtGui.QVBoxLayout()
    vboxMid1.addWidget(vxLabel)
    vboxMid1.addWidget(self.vxSpin)
    vboxMid1.addStretch(1)
    vboxMid1.addWidget(vzLabel)
    vboxMid1.addWidget(self.vzSpin)
    vboxMid1.addStretch(1)
    vboxMid1.addWidget(durLabel)
    vboxMid1.addWidget(self.durSpin)
    vboxMid1.addStretch(1)
    vboxMid1.addWidget(empty)
    #####=======================================================================
    hboxLower = QtGui.QHBoxLayout()
    hboxLower.addStretch(1)
    hboxLower.addWidget(self.tryButton)
    hboxLower.addStretch(1)
    hboxLower.addWidget(loadButton)
    hboxLower.addStretch(1)
    hboxLower.addWidget(saveButton)
    hboxLower.addStretch(1)
    hboxLower.addWidget(clearButton)
    hboxLower.addStretch(1)
    #####=======================================================================
    vboxMid1.addLayout(hboxLower)
    vboxMid2 = QtGui.QGridLayout()
    vboxMid2.addWidget(addAButton,0,0)
    vboxMid2.addWidget(addBButton,0,1)
    vboxMid2.addWidget(redoButton,1,1)
    vboxMid2.addWidget(changeButton,1,0)
    vboxMid2.addWidget(deleteButton,2,1)
    #####=======================================================================
    # upper part: logo + select robot
    hboxMid.addLayout(vboxMid1)
    hboxMid.addLayout(vboxMid2)
    #####=======================================================================
    vboxLeft = QtGui.QVBoxLayout()
    vboxLeft.addWidget(pic, QtCore.Qt.AlignCenter)
    vboxLeft.addStretch(1)
    vboxLeft.addWidget(empty)
    vboxLeft.addStretch(1)
    vboxLeft.addWidget(platfLab,0)
    vboxLeft.addWidget(self.robotCombo,0)
    vboxLeft.addStretch(1)
    vboxLeft.addLayout(hboxMid)
    #####=======================================================================
    hboxGui = QtGui.QHBoxLayout()
    hboxGui.addLayout(vboxLeft)
    hboxGui.addWidget(self.programList)
    #####=======================================================================

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
    item = QtGui.QListWidgetItem("= vor %.2f = dreh %.2f = dauer %.2f = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
    self.vxSpin.value(), self.vzSpin.value(), self.durSpin.value(), self.xAcc, self.rAcc )) # neu
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
    item = QtGui.QListWidgetItem("= vor %.2f = dreh %.2f = dauer %.2f = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
    self.vxSpin.value(), self.vzSpin.value(), self.durSpin.value(), self.xAcc, self.rAcc )) #neu
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
    
    # create publisher to send velocity commands to selected robot platform
    self.pub = rospy.Publisher(topic, Twist, queue_size=100)

    # get filename and open
    dances_path = rospy.get_param('~dances_path')
    fname = ""
    dialog = QtGui.QFileDialog(self)
    dialog.setWindowTitle('Tanz abspielen')
    dialog.setDirectory(dances_path)
    dialog.setNameFilter('*.dc')
    dialog.setFileMode(QtGui.QFileDialog.ExistingFile)
    if dialog.exec_() == QtGui.QDialog.Accepted:
        fname = dialog.selectedFiles()[0]

    if len(fname)>0:
      file = open(fname,"r")
      # read data and tidy up
      lines = file.readlines()
      file.close()

      self.tryButton.setDisabled (True)

      target_speed = 0
      target_turn = 0
      control_speed = 0
      control_turn = 0

      pub_rate = rospy.Rate(5)

      # process data
      for li in lines:
        l=li.split()

        target_speed = float(l[0]) # speed * x
        target_turn  = float(l[1]) # turn * th
        pub_dur = rospy.Duration.from_sec(float(l[2]))
        acc_speed = float(l[3])
        acc_turn  = float(l[4])

        rospy.loginfo ("Zielgeschwindigkeiten: vor = %.2f; dreh = %.2f; Dauer = %.2fs; ", target_speed, target_turn, pub_dur.to_sec());

        start_time = rospy.get_rostime()
        while True:
          pub_rate.sleep() 
          if rospy.get_rostime() >= start_time + pub_dur: 
            #rospy.loginfo(" -> reached duration %f", pub_dur.to_sec())
            break

          # stepwise TODO fix as sequence of commands
          if target_speed > control_speed:
            control_speed = min( target_speed, control_speed + acc_speed )
          elif target_speed < control_speed:
            control_speed = max( target_speed, control_speed - acc_speed )
          else:
            control_speed = target_speed
            #rospy.loginfo ("-> reached target velocity!")
  
          # stepwise TODO fix as sequence of commands
          if target_turn > control_turn:
            control_turn = min( target_turn, control_turn + acc_turn )
          elif target_turn < control_turn:
            control_turn = max( target_turn, control_turn - acc_turn )
          else:
            control_turn = target_turn

          # do the thing
          twist = Twist()
          twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
          twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
          #rospy.loginfo(" publish cmd_vel: control_speed=%f; control_turn=%f", control_speed, control_turn)
          self.pub.publish(twist)

      time.sleep(0.5)
      print "Tanz ist fertig!\n"
      self.tryButton.setDisabled (False)

  ##############################################################################

  # apply change in acceleration 
  def applyAcc( self ): # neu
    #print self.rAccSpin.value(), self.xAccSpin.value()
    # read old values
    ind = self.programList.selectedIndexes()
    if len( ind )>0:
      # split
      s = str(ind[0].data().toString())
      l=s.split()
      # get values
      self.vxSpin.setValue( float( l[2] ) )
      self.vzSpin.setValue( float( l[5] ) )
      self.durSpin.setValue( float( l[8] ) )
    # prepare new step description
    item = QtCore.QString("= vor %s = dreh %s = dauer %s = ( vorBeschl %.2f , drehBeschl %.2f )" %( \
    l[2], l[5], l[8], self.xAccSpin.value(), self.rAccSpin.value() )) 
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
    l=s.split()

    # open new widget
    self.cw = ChangeWid()
    self.cw.setGeometry(QtCore.QRect(750, 250, 200, 200))
    self.cw.setWindowTitle(u'Beschleunigung ändern')
    #self.setFixedSize( self.sizeHint() )
    #self.layout().setSizeConstraint( QtGui.QLayout.SetFixedSize )
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
   
    cwLayout = QtGui.QVBoxLayout()
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

###########################################################

def main():

  app = QtGui.QApplication(sys.argv)
  ex = TurtleDance()
  sys.exit(app.exec_())

if __name__ == '__main__':
  main()
