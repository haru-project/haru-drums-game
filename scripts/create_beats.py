#!/usr/bin/env python3
import rtmidi
import mido
import subprocess
import time
import rospy
import signal
import os
import rospkg
import yaml

class BeatsCreator():

    def __init__(self):
        # Create a MidiIn object
        self.midi_in = rtmidi.MidiIn()
        # Crear el objeto MidiFile
        self.midi_file = mido.MidiFile()
        # Agregar una pista al objeto MidiFile
        self.track = mido.MidiTrack()
        self.midi_file.tracks.append(self.track)
        # List all available MIDI input ports
        ports = self.midi_in.get_ports()
        print("Available MIDI input ports:")
        for i, port in enumerate(ports):
            print(f"{i}: {port}")
        # Choose a MIDI input port to use
        port_index = None
        available_port = False
        print("Select the Midi port you want to use:")
        while not available_port:
            port_index = int(input())
            if port_index <= len(ports) - 1 and port_index >= 0:
                available_port = True
            else:
                print("This is not an available port. Please try again:")
        self.midi_in.open_port(port_index)
        self.now = time.time()
        self.last_note = 0
        self.color_combination = dict()
        self.cont = 0
        drums_game_package_path = rospkg.RosPack().get_path("haru_drums_game")
        drum_drivers_package_path = rospkg.RosPack().get_path("haru_drums_ros_driver")
        config_path = os.path.join(drum_drivers_package_path, "config")
        settings_path = os.path.join(config_path, "drum_settings.yaml") 
        self.midi_files_path = os.path.join(drums_game_package_path, "src/midi_files/")
        self.wav_files_path = os.path.join(drums_game_package_path, "src/wav_files/")
        self.yaml_files_path = os.path.join(drums_game_package_path, "src/yaml_files/")
        with open(settings_path, "r") as file:
            settings = yaml.safe_load(file)
        self.num_to_color_dict = settings["number_color"]
        # Set the callback function for the MidiIn object
        self.midi_in.set_callback(self.handle_message)
        signal.signal(signal.SIGINT, self.shutdown)

    # Define a callback function to handle incoming MIDI messages
    def handle_message(self, event, data):
        message, delta_time = event[0], event[1]
        if delta_time > 0.2 or delta_time==0:
            print("event: ", event)
            elapse_time = int((time.time() - self.now) * 1000) 
            self.now = time.time()
            print("elapse time:", elapse_time)
            if message[0] == 153:
                self.track.append(mido.Message("note_on", note=message[1], velocity=127, time=elapse_time, channel=9))
                self.color_combination[self.cont] = self.num_to_color_dict[message[1]]
                self.cont += 1
            elif message[0] == 137:
                self.track.append(mido.Message("note_off", note=message[1], velocity=127, time=elapse_time, channel=9))
    
    def shutdown(self, signum, frame):
        # Save the midi file and create the wav file
        self.midi_file.save(self.midi_files_path + 'example.mid')
        with open(self.wav_files_path + "example.wav", "wb") as f:
            subprocess.call(["timidity", "-Ow", "-o", "-", self.midi_files_path + 'example.mid'], stdout=f)
        # Close the MIDI input stream
        self.midi_in.close_port()
        del self.midi_in
        with open(self.yaml_files_path + "example.yaml", 'w') as file:
            yaml.dump(self.color_combination, file)
        exit()
    
if __name__ == '__main__':
    rospy.init_node("midi_drum")
    drum = BeatsCreator()
    rospy.spin()