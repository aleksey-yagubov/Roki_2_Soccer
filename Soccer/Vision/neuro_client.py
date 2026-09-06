#!/usr/bin/env python3
"""
Thin client for the external OpenVINO neural worker.

The main robot runtime must not import OpenVINO. This module only writes the
current frame to shared memory and sends a request over a Unix socket.
"""

import json
import logging
import os
import socket
import time
from multiprocessing.shared_memory import SharedMemory

import numpy as np


FRAME_SHAPE = (650, 800, 3)
FRAME_DTYPE = np.uint8
FRAME_SIZE = 650 * 800 * 3
DEFAULT_SHM_NAME = "roki_neuro_frame"
DEFAULT_SOCKET_PATH = "/tmp/roki_neuro.sock"
SUPPORTED_OBJECTS = {"ball", "basket"}


logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
log = logging.getLogger("roki-neuro-client")


class NeuroClient:
    """Drop-in replacement for the old local OpenVINO Neural class."""

    def __init__(
        self,
        role="other",
        display=None,
        shm_name=DEFAULT_SHM_NAME,
        socket_path=DEFAULT_SOCKET_PATH,
        create_shm=True,
    ):
        self.role = role
        self.display = display
        self.shm_name = shm_name
        self.socket_path = socket_path
        self.shm = None
        self.frame_shm = None
        self._owns_shm = False
        self.enabled = True

        try:
            if create_shm:
                try:
                    self.shm = SharedMemory(name=self.shm_name, create=True, size=FRAME_SIZE)
                    self._owns_shm = True
                except FileExistsError:
                    self.shm = SharedMemory(name=self.shm_name, create=False, size=FRAME_SIZE)
            else:
                self.shm = SharedMemory(name=self.shm_name, create=False, size=FRAME_SIZE)

            self.frame_shm = np.ndarray(FRAME_SHAPE, dtype=FRAME_DTYPE, buffer=self.shm.buf)
        except Exception as exc:
            self.enabled = False
            log.warning("Neural worker disabled: cannot attach shared memory %s: %s", self.shm_name, exc)

        if self.enabled and not self.is_ready():
            self.enabled = False
            log.warning("Neural worker disabled: socket %s does not exist", self.socket_path)

    def is_ready(self):
        return self.enabled and os.path.exists(self.socket_path)

    def _send_command(self, cmd):
        for _ in range(20):
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                    sock.settimeout(2.0)
                    sock.connect(self.socket_path)
                    sock.sendall(cmd.encode("utf-8") + b"\n")
                    response = b""
                    while b"\n" not in response:
                        chunk = sock.recv(4096)
                        if not chunk:
                            break
                        response += chunk
                if not response:
                    raise RuntimeError("empty response from neural worker")
                parsed = json.loads(response.decode("utf-8").strip())
                if parsed.get("status") == "error":
                    raise RuntimeError(parsed.get("msg", "neural worker error"))
                return parsed
            except (ConnectionRefusedError, FileNotFoundError):
                time.sleep(0.1)
            except socket.timeout:
                time.sleep(0.1)
        raise RuntimeError("Cannot connect to neural worker at " + self.socket_path)

    def _write_frame(self, frame):
        if frame.shape != FRAME_SHAPE:
            raise ValueError(f"Expected frame shape {FRAME_SHAPE}, got {frame.shape}")
        if frame.dtype != FRAME_DTYPE:
            raise ValueError(f"Expected frame dtype {FRAME_DTYPE}, got {frame.dtype}")
        self.frame_shm[:] = frame

    def ball_detect_single(self, frame):
        return self.object_detect_single(frame, "ball")

    def basket_detect_single(self, frame):
        return self.object_detect_single(frame, "basket")

    def object_detect_single(self, frame, obj):
        if not self.enabled:
            return 0, 0
        if obj not in SUPPORTED_OBJECTS:
            obj = "ball"
        try:
            self._write_frame(frame)
            response = self._send_command("PROCESS " + obj)
            return response.get("cx", 0), response.get("cy", 0)
        except Exception as exc:
            self.enabled = False
            log.warning("Neural worker disabled after inference error: %s", exc)
            return 0, 0

    def close(self, unlink=False):
        if self.shm is None:
            return
        self.shm.close()
        if unlink and self._owns_shm:
            try:
                self.shm.unlink()
            except FileNotFoundError:
                pass
        self.shm = None
        self.frame_shm = None

    def stop(self):
        """Stop the external worker. Use only when shutting down that service."""
        try:
            self._send_command("STOP")
        finally:
            self.close(unlink=True)

    def __del__(self):
        try:
            self.close(unlink=False)
        except Exception:
            pass


class Neural(NeuroClient):
    pass
