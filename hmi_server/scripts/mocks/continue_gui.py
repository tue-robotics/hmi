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


def get_word_options(text):
    """ Returns the possible options that can follow on the provided text

    :param text: string with the currently provided text
    :return: list with possibilities
    """
    print "Text: '{0}'".format(text)
    if text == "":
        return ['go', 'move', 'drive', 'navigate']
    if text in ['go', 'move', 'drive', 'navigate']:
        return ['to']
    if text in ['go to', 'move to', 'drive to', 'navigate to']:
        return ['the']
    if text in ['go to the', 'move to the', 'drive to the', 'navigate to the']:
        return ['livingroom', 'kitchen', 'bedroom', 'hallway']
    else:
        return []


# -----------------------------------------------------------------------------


class UpdateThread(QtCore.QThread):
    """ Update thread """
    description = QtCore.pyqtSignal(['QString'])
    spec = QtCore.pyqtSignal(['QString'])
    key = QtCore.pyqtSignal(['QString'])
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

        self._description = ""
        self._spec = ""
        self._buttons = ""
        self._key = ""
        self._current_text = ""

    def run(self):

        print "Thread.run"
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # question, buttons = self._server.update(self._current_text)
            res = self._server.update(self._current_text)
            description = res.description
            spec = res.spec
            key = res.key
            buttons = res.buttons

            # if question and question != self.question:
            if description != self._description:
                self.description.emit(description)
                self._description = description
            if key != self._key:
                self.key.emit(key)
                self._key = key
            if spec != self._spec:
                self.spec.emit(spec)
                self._spec = spec
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


class UpdateResult(object):
    """ Return value of the update function of the HMIServerGuiInterface """
    def __init__(self, description="", spec="", key="", buttons=[]):
        """ Constructor

        :param description: asked question ("Which <fruit> would you like")
        :param spec: spec ("I would like <fruit>")
        :param key: part of the answer we are answering  ("<fruit>")
        :param buttons: possible buttons to click (["banana", "apple"])
        """
        self.description = description
        self.spec = spec
        self.key = key
        self.buttons = buttons

    def __repr__(self):
        return "Description: {0}\nSpec: {1}\nKey: {2}\nButtons: {3}".format(self.description, self.spec,
                                                                            self.key, self.buttons)


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

        r = rospy.Rate(5.0)
        while self._mode != GuiMode.RESULT_PENDING:  # and not is_preempt_requested()
            if is_preempt_requested():
                break
            # print "Checking result, mode: {0}, preempt requested: {1}".format(self._mode, is_preempt_requested())
            r.sleep()

        self._mode = GuiMode.IDLE

        print "Yeah, we're done, result: {0}".format(self._results_dict)
        return self._result

    def update(self, current_text):
        """ Continuously updates

        :param current_text: current text of the GUI textbox
        :return: tuple with a question (optionally) and a list with the possible words. If the list is empty,
        there is no current goal any more
        """
        # If idle: return
        if self._mode in [GuiMode.IDLE, GuiMode.RESULT_PENDING]:
            return UpdateResult("", "", "", [])

        # If simple question: check if answered
        if self._mode == GuiMode.SIMPLE_QUESTION:
            k = self._choices.keys()[self._simple_question_index]
            v = self._choices[k]

            # ToDo: check if answered
            if current_text in v:
                self._results_dict[k] = current_text
                self._simple_question_index += 1
                if len(self._choices) == self._simple_question_index:
                    self._result = HMIResult(results=self._results_dict)
                    self._mode = GuiMode.RESULT_PENDING
                    return UpdateResult("", "", "", [])
                else:
                    return UpdateResult(self._description, self._spec, "", [])

            return UpdateResult(self._description, self._spec, k, v)

        # If grammar: check parser
        if self._mode == GuiMode.USE_GRAMMAR:
            options = get_word_options(current_text)
            print "Options: {0}".format(options)
            return UpdateResult(self._description, "", "", options)

        return UpdateResult("", "", "", [])


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

        self.resize(640, 480)
        self.move(300, 300)
        self.setWindowTitle('Continue GUI')

        # Add the description box
        self.description_label = QtGui.QLabel(self)
        self.description_label.setMaximumHeight(100)

        # Add the question box
        self.spec_label = QtGui.QLabel(self)
        self.spec_label.setMaximumHeight(100)

        # Add the key box
        self.key_label = QtGui.QLabel(self)
        self.key_label.setMaximumHeight(100)

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
        self._main_layout.addWidget(self.description_label)
        self._main_layout.addWidget(self.spec_label)
        self._main_layout.addWidget(self.key_label)
        self._main_layout.addWidget(self._top_widget)
        # self._main_layout.addWidget(self.option_buttons)
        self._main_layout.addWidget(self.button_widget)
        self._main_layout.insertStretch(-1, 0)

        # List with ButtonCBs
        self._button_cbs = []

        # Setup the interface
        self.server_interface = HMIServerGUIInterface('blaat')

        # Update thread
        self.update_thread = UpdateThread(self, self.server_interface)
        self.update_thread.buttons.connect(self.buttons_callback)
        self.update_thread.description.connect(self.description_callback)
        self.update_thread.spec.connect(self.spec_callback)
        self.update_thread.key.connect(self.key_callback)
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

    def description_callback(self, text):
        """ Puts the provided text in the label
        :param text: string with text
        """
        self.description_label.setText(text)

    def key_callback(self, text):
        """ Puts the provided text in the label
        :param text: string with text
        """
        self.key_label.setText(text)

    def spec_callback(self, text):
        """ Puts the provided text in the label
        :param text: string with text
        """
        self.spec_label.setText(text)

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
        """ PyQt slot for buttons to add. Buttons are always cleared beforehand
        :param buttons: QString containing the text of the buttons to add, separated by ';'
        """
        self.clear_buttons()
        if buttons == "":
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
