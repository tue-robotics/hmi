#!/usr/bin/python


import sys

# ROS
import rospy

# Qt
from python_qt_binding import QtGui
from python_qt_binding import QtCore

# TU/e
from hmi_server.abstract_server import HMIResult, AbstractHMIServer

# -----------------------------------------------------------------------------


class UpdateThread(QtCore.QThread):
    """ Update thread """
    question = QtCore.pyqtSignal(['QString'])
    buttons = QtCore.pyqtSignal(['QString'])  # String, buttons separated by ;

    def __init__(self, gui, server):
        """ Constructor

        :param gui: GUI where things are displayed
        :param server: hmi server interface
        """
        super(UpdateThread, self).__init__()
        print "Thread init"
        self._gui = gui
        self._server = server

        self._buttons = ""
        self._question = ""
        self._current_text = ""

    def run(self):

        print "Thread.run"
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            question, buttons = self._server.update(self._current_text)

            # if question and question != self.question:
            if question != self.question:
                self.question.emit(question)
                self._question = question
            # if len(buttons) > 0:
            #     buttonstr = ""
            #     for b in buttons:
            #         buttonstr += (b + ";")
            #     if buttonstr != self._buttons:
            #         self.buttons.emit(buttonstr)
            #         self._buttons = buttonstr
            buttonstr = ""
            for b in buttons:
                buttonstr += (b + ";")
            if buttonstr != self._buttons:
                self.buttons.emit(buttonstr)
                self._buttons = buttonstr
            r.sleep()

    def set_text(self, text):
        """ Sets the text to use in the update loop of the HMI server
        :param text: text to set
        """
        self._current_text = str(text)


# -----------------------------------------------------------------------------


class GuiMode(object):
    RESULT_PENDING = -1
    IDLE = 0
    SIMPLE_QUESTION = 1
    USE_GRAMMAR = 2


# -----------------------------------------------------------------------------


class HMIServerGUIInterface(AbstractHMIServer):
    """ Provides the HMI server within the ContinueGUI. Contains '_determine_answer'
    only sets the goal. An 'update' function, that must be called in a loop, does
    the actual work
    """
    def __init__(self, name):
        """ Constructor
        :param name: ???
        """
        super(HMIServerGUIInterface, self).__init__(name)

        self._description = ""
        self._choices = {}
        self._spec = ""
        self._is_preempt_requested = False

        self._mode = GuiMode.IDLE

        self._result = None  # HMIResult(results=results)

        # For simple questions
        self._simple_question_index = 0
        self._results_dict = {}

    def _determine_answer(self, description, spec, choices, is_preempt_requested):
        """
        Sets some members and blocks until we can either an answer is created
        or the goal is cancelled

        Return the answer
        Return None if nothing is heared
        Raise an Exception if an error occured
        """
        self._description = description
        self._spec = spec
        self._choices = choices
        self._is_preempt_requested = is_preempt_requested

        if self._choices:
            self._mode = GuiMode.SIMPLE_QUESTION
            self._simple_question_index = 0
            self._results_dict = {}
        else:
            self._mode = GuiMode.USE_GRAMMAR


        #####
        print "Yeah, received a request"

        r = rospy.Rate(0.2)
        while self._mode != GuiMode.RESULT_PENDING:
            r.sleep()

        self._mode = GuiMode.IDLE

        return self._result

    def update(self, current_text):
        """ Continuously updates

        :param current_text: current text of the GUI textbox
        :return: tuple with a question (optionally) and a list with the possible words. If the list is empty,
        there is no current goal any more
        """
        # If idle: return
        if self._mode in [GuiMode.IDLE, GuiMode.RESULT_PENDING]:
            return "", []

        # If simple question: check if answered
        if self._mode == GuiMode.SIMPLE_QUESTION:
            k = self._choices.keys()[self._simple_question_index]
            v = self._choices[k]

            # ToDo: check if answered
            if current_text in v:
                self._results_dict[k] = current_text
                self._simple_question_index += 1
                if len(self._choices) == self._simple_question_index:
                    print "Yeah, we're done, result: {0}".format(self._results_dict)
                    self._result = HMIResult(results=self._results_dict)
                    self._mode = GuiMode.RESULT_PENDING
                    return "", []

            return k, v


# -----------------------------------------------------------------------------


class ButtonCB(object):
    """ Helper object to dynamically create callback functions """
    def __init__(self, parent, text):
        self._parent = parent
        self._text = text

    def callback(self):
        self._parent.add_to_textbox(self._text)


# -----------------------------------------------------------------------------


class ContinueGui(QtGui.QWidget):
    """
    GUI for the RoboCup continue rule
    """
    current_text = QtCore.pyqtSignal(['QString'])   # Necessary to emit current text to update thread

    def __init__(self):
        """ Constructor """
        super(ContinueGui, self).__init__()
        # ToDo: add actionlib client

        self.resize(640, 480)
        self.move(300, 300)
        self.setWindowTitle('Continue GUI')

        # Add the question box
        self.question_label = QtGui.QLabel(self)
        self.question_label.setMaximumHeight(100)

        # Add the texteditbox
        self.textbox = QtGui.QTextEdit(self)
        self.textbox.textChanged.connect(self._text_changed)

        # Add the submit button
        self.submit_button = QtGui.QPushButton(self)
        self.submit_button.setText("Submit")
        # ToDo: add callback

        # Add the QButtonGroup
        # self.option_buttons = QtGui.QButtonGroup(self) (ToDo: how can I do this nicely???)
        self.button_layout = QtGui.QVBoxLayout()
        self.button_widget = QtGui.QWidget()
        self.button_widget.setLayout(self.button_layout)
        self.button_widget.setSizePolicy(2, 2)  # Makes sure we don't get enormous buttons

        # Top layout
        self._top_layout = QtGui.QHBoxLayout(self)
        self._top_layout.addWidget(self.textbox)
        self._top_layout.addWidget(self.submit_button)
        self._top_widget = QtGui.QWidget()
        self._top_widget.setLayout(self._top_layout)
        self._top_widget.setMaximumHeight(100)

        # Main layout
        self._main_layout = QtGui.QVBoxLayout(self)
        self._main_layout.addWidget(self.question_label)
        self._main_layout.addWidget(self._top_widget)
        # self._main_layout.addWidget(self.option_buttons)
        self._main_layout.addWidget(self.button_widget)

        # List with ButtonCBs
        self._button_cbs = []

        # Setup the interface
        self.server_interface = HMIServerGUIInterface('blaat')

        # Update thread
        self.update_thread = UpdateThread(self, self.server_interface)
        self.update_thread.buttons.connect(self.buttons_callback)
        self.update_thread.question.connect(self.label_callback)
        self.current_text.connect(self.update_thread.set_text)
        self.update_thread.start()

        self.show()

    def _text_changed(self):
        """ Callback function if text in the textbox has changed. Gets the text and emits
        this as a signal
        """
        text = self.textbox.toPlainText()
        print "Emitting: {0}".format(text)
        self.current_text.emit(text)

    def label_callback(self, text):
        """ Puts the provided text in the label
        :param text: string with text
        """
        self.question_label.setText(text)

    def add_to_textbox(self, text):
        """ Adds the text to the textbox, taking spaces into account

        :param text: text to add
        :return:
        """
        current_text = self.textbox.toPlainText()
        if current_text == "":
            self.textbox.insertPlainText(text)
        else:
            self.textbox.insertPlainText(" "+text)

    def buttons_callback(self, buttons):
        """ PyQt slot for buttons to add
        :param buttons: QString containing the text of the buttons to add, separated by ';'
        """
        if buttons == "":
            self.clear_buttons()
            self.clear_text()

        for b in str(buttons).split(';'):
            self.add_button(b)

    def add_button(self, text):
        """ Adds a button with an option to the GUI
        :param text: text of the button to add
        """
        if text == "":
            return
            # self.clear_buttons()

        b = QtGui.QPushButton()
        b.setText(text)

        bcb = ButtonCB(self, text)
        self._button_cbs.append(bcb)
        self.button_layout.addWidget(b)
        b.clicked.connect(bcb.callback)

    def clear_buttons(self):
        """ Removes all option buttons """
        # Remove all buttons
        while True:
            b = self.button_layout.takeAt(0)
            if b is None:
                break
            else:
                b.widget().setParent(None)

        # Clear the list with callbacks
        self._button_cbs = []

    def clear_text(self):
        """ Clears the text from the textbox """
        self.textbox.setText("")


if __name__ == '__main__':

    rospy.init_node('continue_gui', anonymous=True)

    # For proper closing?
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    #####

    app = QtGui.QApplication(sys.argv)

    w = ContinueGui()

    sys.exit(app.exec_())
