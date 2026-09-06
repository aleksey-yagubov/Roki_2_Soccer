import os
import re
import threading
import time

import cv2
import numpy as np


def _stream_name(name):
    if not name:
        return "main"
    return re.sub(r"[^a-zA-Z0-9_]+", "_", str(name)).strip("_").lower() or "main"


class NullDisplay:
    backend_name = "null"

    def show(self, name, frame):
        return False

    def close(self):
        pass


class OpenCvDisplay:
    backend_name = "imshow"

    def __init__(self, wait_key_ms=1):
        self.wait_key_ms = wait_key_ms

    def show(self, name, frame):
        if frame is None:
            return False
        cv2.imshow(str(name), frame)
        cv2.waitKey(self.wait_key_ms)
        return True

    def close(self):
        cv2.destroyAllWindows()


class RtspDisplay:
    backend_name = "rtsp"

    class _Stream:
        def __init__(self, display, name, width, height, fmt):
            self.display = display
            self.name = name
            self.width = width
            self.height = height
            self.fmt = fmt
            self.base_time_ns = None
            self.appsrc = None
            self.prepared = False
            self.lock = threading.Lock()
            self.mount_path = "/" + _stream_name(name)
            caps = f"video/x-raw,format={fmt},width={width},height={height}"
            launch = (
                f'appsrc name=source is-live=true block=false format=time do-timestamp=true caps="{caps}" '
                "! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
                "! videoconvert ! v4l2jpegenc ! rtpjpegpay name=pay0 pt=26"
            )
            factory = display.GstRtspServer.RTSPMediaFactory()
            factory.set_shared(True)
            factory.set_launch(launch)
            factory.connect("media-configure", self._on_media_configure)
            self.factory = factory

        def _on_media_configure(self, factory, media):
            media.connect("prepared", self._on_prepared)
            media.connect("unprepared", self._on_unprepared)
            element = media.get_element()
            if element is None:
                self.display._disable(f"RTSP stream {self.name} media element creation failed")
                return
            bus = element.get_bus()
            if bus is not None:
                bus.add_signal_watch()
                bus.connect("message::error", self._on_bus_error)
            appsrc = element.get_child_by_name("source")
            if appsrc is None:
                self.display._disable(f"RTSP stream {self.name} appsrc not found")
                return
            appsrc.set_property("is-live", True)
            appsrc.set_property("block", False)
            appsrc.set_property("format", self.display.Gst.Format.TIME)
            appsrc.set_property("do-timestamp", True)
            with self.lock:
                self.appsrc = appsrc
                self.base_time_ns = None
                self.prepared = False

        def _on_bus_error(self, bus, message):
            err, debug = message.parse_error()
            details = str(err)
            if debug:
                details = f"{details} ({debug})"
            self.display._disable(f"RTSP stream {self.name} error: {details}")

        def _on_prepared(self, media):
            with self.lock:
                self.prepared = True
                self.base_time_ns = None

        def _on_unprepared(self, media):
            with self.lock:
                self.prepared = False
                self.appsrc = None
                self.base_time_ns = None

        def push(self, frame):
            with self.lock:
                if self.appsrc is None or not self.prepared:
                    return False
                if self.base_time_ns is None:
                    self.base_time_ns = time.monotonic_ns()
                pts = time.monotonic_ns() - self.base_time_ns
                data = np.ascontiguousarray(frame)
                buffer = self.display.Gst.Buffer.new_allocate(None, data.nbytes, None)
                buffer.fill(0, data.tobytes())
                buffer.pts = pts
                buffer.dts = pts
                result = self.appsrc.emit("push-buffer", buffer)
                return result == self.display.Gst.FlowReturn.OK

    def __init__(self, port=8554):
        import gi

        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtspServer", "1.0")
        from gi.repository import GLib, Gst, GstRtspServer

        self.GLib = GLib
        self.Gst = Gst
        self.GstRtspServer = GstRtspServer
        self.Gst.init(None)
        self.port = str(port)
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(self.port)
        self.mounts = self.server.get_mount_points()
        self.server.attach(None)
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.loop_thread.start()
        self.streams = {}
        self.streams_lock = threading.Lock()
        self.disabled = False
        self.disable_lock = threading.Lock()

    def _disable(self, reason):
        with self.disable_lock:
            if self.disabled:
                return
            self.disabled = True
            print(reason, flush=True)
            print("RTSP display disabled, falling back to null backend", flush=True)

    def _frame_format(self, frame):
        if frame.ndim == 2:
            return frame.shape[1], frame.shape[0], "GRAY8"
        if frame.ndim == 3 and frame.shape[2] == 3:
            return frame.shape[1], frame.shape[0], "BGR"
        return None

    def _get_stream(self, name, frame):
        if self.disabled:
            return None
        stream_name = _stream_name(name)
        fmt = self._frame_format(frame)
        if fmt is None:
            return None
        width, height, pixel_format = fmt
        with self.streams_lock:
            stream = self.streams.get(stream_name)
            if stream is not None:
                if (
                    stream.width != width
                    or stream.height != height
                    or stream.fmt != pixel_format
                ):
                    self._disable(
                        f"RTSP stream {stream_name} format changed from "
                        f"{stream.width}x{stream.height} {stream.fmt} to "
                        f"{width}x{height} {pixel_format}"
                    )
                    return None
                return stream
            try:
                stream = self._Stream(self, stream_name, width, height, pixel_format)
                self.mounts.add_factory(stream.mount_path, stream.factory)
                self.streams[stream_name] = stream
                print(f"RTSP stream {stream_name} started", flush=True)
                return stream
            except Exception as exc:
                self._disable(f"RTSP stream {stream_name} init failed: {exc}")
                return None

    def show(self, name, frame):
        if frame is None or self.disabled:
            return False
        stream = self._get_stream(name, frame)
        if stream is None:
            return False
        try:
            return stream.push(frame)
        except Exception as exc:
            self._disable(f"RTSP stream {stream.name} push failed: {exc}")
            return False

    def close(self):
        self.loop.quit()


def create_display(params, simulation):
    backend = os.environ.get("ROKI_DISPLAY_BACKEND", params.get("DISPLAY_BACKEND", ""))
    backend = str(backend).strip().lower()
    if not backend:
        backend = "null" if simulation == 5 else "imshow"
    if backend == "imshow":
        return OpenCvDisplay(wait_key_ms=int(params.get("DISPLAY_WAIT_KEY_MS", 1)))
    if backend == "rtsp":
        try:
            return RtspDisplay(port=int(params.get("DISPLAY_RTSP_PORT", 8554)))
        except Exception as exc:
            print(f"RTSP display init failed: {exc}")
            return NullDisplay()
    return NullDisplay()
