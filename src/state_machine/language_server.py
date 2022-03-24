import time
import json
import numpy as np
import speech_recognition as sr
from urllib.request import urlopen
import pyttsx3


class LanguageServer:

    def __init__(self, file_path='mic_config.json'):
        self.file_path = file_path
        self.config = self._load_config()
        #Load YOLO
        self.r, self.asocket = self._load_network_properties()
        self.engine = pyttsx3.init()
        self.engine.setProperty('voice', 'english_rp+f3')  #my preference
        self.available_corners = [
            'bottom left', 'bottom right', ' top left', ' top right'
        ]
        self.available_answers = ['yes', 'no', 'close', 'open']

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
            print("Say something!")
            # self.r.adjust_for_ambient_noise(source, duration=1)
            audio = self.r.listen(source, phrase_time_limit=5)
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
    def get_corner_id_voice(self):
        while True:
            audio = self.get_audio()
            recognized_str = str(self.recognize(audio))
            print(recognized_str)
            self.say_word(recognized_str)
            # synthesize_text(recognized_str)
            # time.sleep(1)


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
    sp.broadcast()
    # synthesize_text("hello")