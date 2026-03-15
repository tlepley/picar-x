#!/usr/bin/env python3
"""Remote text+vision LLM app using snapshots from the ROS image stream."""

from __future__ import annotations

import os
import tempfile

import cv2

from picarx.llm import Ollama

from ..remote_lib.remote_camera_stream import RemoteCameraStream


def main() -> None:
    instructions = 'You are a helpful assistant.'
    welcome = 'Hello, I am a helpful assistant. How can I help you?'
    llm = Ollama(ip='192.168.100.145', model='llava:7b')
    llm.set_max_messages(20)
    llm.set_instructions(instructions)
    llm.set_welcome(welcome)
    stream = RemoteCameraStream()
    if not stream.request_start():
        raise SystemExit('Could not start remote camera stream')
    print(welcome)
    try:
        while True:
            input_text = input('>>> ')
            if input_text.strip().lower() in {'exit', 'quit'}:
                break
            frame = stream.latest_frame()
            if frame is None:
                print('No frame available yet.')
                continue
            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
                img_path = tmp.name
            cv2.imwrite(img_path, frame)
            try:
                response = llm.prompt(input_text, stream=True, image_path=img_path)
                for next_word in response:
                    if next_word:
                        print(next_word, end='', flush=True)
                print('')
            finally:
                if os.path.exists(img_path):
                    os.unlink(img_path)
    finally:
        stream.request_stop()
        stream.close()
