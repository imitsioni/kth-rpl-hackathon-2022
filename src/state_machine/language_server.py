from datetime import date
import time
import json
import numpy as np
import speech_recognition as sr
#import playsound
# from urllib.request import urlopen
# from urllib2 import urlopen
from future.moves.urllib.request import urlopen, Request
import pyttsx3
import rospy

from playsound import playsound

from threading import Lock, Thread


class LanguageServer:

    def __init__(self,
                 file_path='mic_config.json',
                 folder_path='Salli',
                 folder_path_kid='Ivy',
                 flag_kid_voice=True):
        self.file_path = file_path
        self.config = self._load_config()
        self.playsound = playsound
        self.voice_path = folder_path if not flag_kid_voice else folder_path_kid
        #Load YOLO
        self.r, self.asocket = self._load_network_properties()
        self.engine = pyttsx3.init()
        self.engine.setProperty('voice', 'english_rp+f3')  #my preference
        self.available_corners = [
            'bottom left', 'bottom right', ' top left', ' top right'
        ]
        self.available_answers = ['yes', 'no', 'close', 'open']

        self.stopping_thread = Thread(target=self.stop_thread).start()
        self.stop = False

    def stop_thread(self):
        while not self.stop:
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            if 'stop' in recognized_str:
                self.playsound(self.voice_path + '/' + 'shut_down.mp3')
                rospy.signal_shutdown("stopped by user")
                break

        # self.behaviours = {
        #     "movie": self.process_movie,
        #     "repeat_review": self.process_repeat_review,
        #     "sentiment": self.process_sentiment,
        #     "choose": self.process_choice,
        #     "opinion": self.process_opinion
        # }

    def _load_network_properties(self):
        audio_socket = 'http://' + self.config["Audio_IP"] + ':' + self.config[
            "audio_port"] + '/audio.wav'
        recognizer = sr.Recognizer()

        return recognizer, audio_socket

    def _load_config(self):
        # Load configuration
        with open(self.file_path) as f:
            config = json.load(f)
        print(config)
        return config

    def process_speech(self):
        recognized_word = self.get_mic_input()
        self.movie = recognized_word
        print("SERVER: word is {}".format(self.movie))

    def process_review(self):
        recognized_str = self.get_mic_input()
        self.sentiment = recognized_str
        print("SERVER: user feedback is {}".format(self.sentiment))

    # def process_sentiment(self):
    #     sentiment_score = self.senti_analysist.predict(self.sentiment)
    #     print(sentiment_score[0][0])

    #     if 0 < sentiment_score[0][0] <= 0.15:
    #         preview_sentiment = 'bad'
    #     elif 0.15 < sentiment_score[0][0] <= 0.40:
    #         preview_sentiment = 'slightly bad'
    #     elif 0.40 < sentiment_score[0][0] <= 0.60:
    #         preview_sentiment = 'average'
    #     elif 0.60 < sentiment_score[0][0] <= 0.80:
    #         preview_sentiment = 'pretty good'
    #     elif 0.80 < sentiment_score[0][0] <= 1:
    #         preview_sentiment = 'almost perfect'

    #     rating = round(sentiment_score[0][0] * 10)
    #     preview_sentiment = preview_sentiment + " with a rating of {} out of 10".format(
    #         rating)

    #     print("SERVER: sentiment analysis is {} on {}".format(
    #         preview_sentiment, sentiment_score[0]))

    #     self.outsocket.send_string(preview_sentiment)

    def get_mic_input(self):
        recognized_str = 'None'
        while recognized_str == 'None':
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            if recognized_str != 'None':
                print(recognized_str)
        return recognized_str

    def get_audio(self):
        with sr.WavFile(urlopen(self.asocket)) as source:
            # print("Say something!")
            # self.r.adjust_for_ambient_noise(source, duration=1)
            audio = self.r.listen(source, phrase_time_limit=3)
        return audio

    def recognize(self, audio):
        word = None
        try:
            word = self.r.recognize_google(audio)
        except sr.UnknownValueError:
            pass
            # print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print(
                "Could not request results from Google Speech Recognition service; {0}"
                .format(e))
        return word

    def say_word(self, word):
        self.engine.say(word)
        self.engine.runAndWait()

    def broadcast(self):
        while True:
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            print(recognized_str)
            self.say_word(recognized_str)
            # synthesize_text(recognized_str)
            # time.sleep(1)

    ##################
    ##### BAXTER #####
    ##################
    def list_answer_options_corners(self):
        str_answer = 'Available corner options are:    '
        for _ in self.available_corners:
            str_answer += _ + " or "
        print(str_answer)
        self.say_word(str_answer)

    def list_available_answers(self):
        str_answer = 'Available answers are: '
        for _ in self.available_answers:
            str_answer += _ + " or "
        print(str_answer)
        self.say_word(str_answer)

    def get_corner_id_voice(self):
        while True:
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            print(recognized_str)
            self.say_word(recognized_str)
        return corner_id
        # synthesize_text(recognized_str)
        # time.sleep(1)

    def ask_if_folding_correct(self):

        self.say_word("Is the folding for this corner correct?")

    def ask_for_folding_correction(self):
        self.say_word("Please correct the folding of the current corner")

    def plese_propose_corner(self):
        #self.say_word('Please propose a corner!')
        self.playsound(self.voice_path + '/' + 'please_corner.mp3')

    def ask_for_start_grasping(self):
        while True:
            self.say_word("Would you for me to start grasping?")
            rospy.loginfo("Baxter: Can I Start grasping?")
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            rospy.loginfo('String detected: {}'.format(recognized_str))
            if 'yes' in recognized_str:
                self.say_word("Starting grasping")
                rospy.loginfo("Baxter: Starting grasping")
                break

            if recognized_str in self.available_answers:
                if recognized_str == 'no':
                    self.say_word("Standing still")
                    time.sleep(5)
                elif recognized_str == 'close' or recognized_str == 'open':
                    self.say_word("Did I ask anything about the gripper?")
                else:
                    self.say_word("Starting grasping")
                    rospy.loginfo("Baxter: Starting grasping")
                    break
            else:
                #self.say_word("Did not understand your answer")
                self.playsound(self.voice_path + '/' +
                               'did_not_understand_answer')
                self.list_available_answers()
        return True, recognized_str

    def ask_for_close_gripper(self):
        while True:
            ##self.say_word("Can I close the gripper?")
            self.playsound(self.voice_path + '/' +
                           'should_I_close_gripper.mp3')
            rospy.loginfo("Baxter: Can I close the gripper?")
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            rospy.loginfo('String detected: {}'.format(recognized_str))
            if 'yes' in recognized_str:
                #self.say_word("Closing the gripper")
                self.playsound(self.voice_path + '/' + 'closing_gripper.mp3')
                rospy.loginfo("Baxter: Closing the gripper")
                break

            if recognized_str in self.available_answers:
                if recognized_str == 'no' or recognized_str == 'open':
                    #self.say_word("Opening the Gripper")
                    self.playsound(self.voice_path + '/' +
                                   'opening_gripper.mp3')

                else:
                    #self.say_word("Closing the gripper")
                    self.playsound(self.voice_path + '/' +
                                   'closing_gripper.mp3')
                    rospy.loginfo("Baxter: Closing the gripper")
                    break

            else:
                #self.say_word("Did not understand your answer")
                self.playsound(self.voice_path + '/' +
                               'did_not_understand_answer')
                self.list_available_answers()
        return True, recognized_str

    def ask_for_corner(self):
        #self.say_word("Do you want to propose a corner for folding?")
        self.playsound(self.voice_path + '/' + 'do_want_propose_corner.mp3')

        rospy.loginfo("Baxter: Grasp Another Corner?")
        while True:
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            rospy.loginfo('String detected: {}'.format(recognized_str))
            if 'yes' in recognized_str:
                #self.say_word("Please propose a corner")
                #self.playsound(self.voice_path + '/' + 'please_corner.mp3')

                rospy.loginfo("Baxter: Waiting for you!")
                break

            if recognized_str in self.available_answers:
                if recognized_str == 'no':
                    self.say_word("Standing still")
                    time.sleep(5)
                elif recognized_str == 'close' or recognized_str == 'open':
                    #self.say_word("Did I ask anything about the gripper?")
                    self.playsound(self.voice_path + '/' +
                                   'did_not_understand_answer')
                else:
                    #self.say_word("Please propose a corner")
                    self.playsound(self.voice_path + '/' + 'please_corner.mp3')

                    rospy.loginfo("Baxter: Waiting for you!")
                    break

            else:
                #self.say_word("Did not understand your answer")
                self.playsound(self.voice_path + '/' +
                               'did_not_understand_answer')
                self.list_available_answers()
        return True, recognized_str

    def thanking_for_participating(self):
        #self.say_word("Thank you for participating in the folding with Baxter")
        self.playsound(self.voice_path + '/' + 'thanks.mp3')


def synthesize_text(text):
    """Synthesizes speech from the input string of text."""
    from google.cloud import texttospeech

    client = texttospeech.TextToSpeechClient()

    input_text = texttospeech.SynthesisInput(text=text)

    # Note: the voice can also be specified by name.
    # Names of voices can be retrieved with client.list_voices().
    voice = texttospeech.VoiceSelectionParams(
        language_code="en-US",
        name="en-US-Standard-C",
        ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
    )

    audio_config = texttospeech.AudioConfig(
        audio_encoding=texttospeech.AudioEncoding.MP3)

    response = client.synthesize_speech(request={
        "input": input_text,
        "voice": voice,
        "audio_config": audio_config
    })

    # The response's audio_content is binary.
    with open("output.mp3", "wb") as out:
        out.write(response.audio_content)
        print('Audio content written to file "output.mp3"')


if __name__ == "__main__":
    sp = LanguageServer()
    # sp.say_word("How")
    # sp.say_word("are")
    # sp.say_word("you doing?")LanguageServer
    # sp.ask_for_start_grasping()
    # sp.ask_if_folding_correct()
    # sp.ask_for_folding_correction()
    sp.broadcast()
    # sp.say_word("How")
    # sp.say_word("are")
    # sp.say_word("you doing?")
    # sp.list_answer_options_corners()
    # sp.list_available_answers()
    # sp.ask_for_close_gripper()
    # sp.ask_for_start_grasping()
    # sp.ask_if_folding_correct()
    # sp.ask_for_folding_correction()
    #sp.broadcast()
    # synthesize_text("hello")