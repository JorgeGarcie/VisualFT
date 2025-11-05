#!/usr/bin/env python3

import cv2
import subprocess
import numpy as np
import threading
from queue import Queue
import time
import sys
#works but has some issues with latency and frame drops
class GStreamerOpenCV:
    def __init__(self):
        self.frame_queue = Queue(maxsize=3)  # Smaller queue for less latency
        self.process = None
        self.running = False
        self.width = 1920
        self.height = 1080
        self.frame_size = self.width * self.height * 3  # BGR format
        self.frames_received = 0
        
    def start_stream(self):
        # GStreamer command that outputs raw BGR video to stdout
        cmd = [
            'gst-launch-1.0', '-q',  # -q for quiet mode
            'udpsrc', 'port=5000',
            'caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000',
            '!', 'rtpjitterbuffer', 'latency=0',
            '!', 'rtph264depay',
            '!', 'avdec_h264',
            # '!', 'h264parse',

            '!', 'videoconvert',
            '!', 'video/x-raw,format=BGR,width=1920,height=1080',
            '!', 'fdsink', 'fd=1'  # Output to stdout
        ]
        
        print("Starting GStreamer pipeline for 1920x1080 stream...")
        print("Command:", ' '.join(cmd))
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            
            self.running = True
            
            # Start frame reading thread
            thread = threading.Thread(target=self._read_frames, daemon=True)
            thread.start()
            
            # Start error monitoring thread
            error_thread = threading.Thread(target=self._monitor_errors, daemon=True)
            error_thread.start()
            
            return True
            
        except Exception as e:
            print(f"Failed to start GStreamer process: {e}")
            return False
    
    def _monitor_errors(self):
        """Monitor GStreamer errors"""
        while self.running and self.process:
            try:
                line = self.process.stderr.readline()
                if line:
                    error_msg = line.decode().strip()
                    if error_msg:  # Only print non-empty errors
                        print(f"GStreamer: {error_msg}")
            except:
                break
    
    def _read_frames(self):
        """Read raw frames from GStreamer stdout"""
        print("Starting frame reader thread...")
        buffer = b''
        
        while self.running and self.process:
            try:
                # Read data in chunks
                chunk = self.process.stdout.read(65536)  # 64KB chunks
                if not chunk:
                    print("No more data from GStreamer")
                    break
                
                buffer += chunk
                
                # Process complete frames
                while len(buffer) >= self.frame_size:
                    # Extract one frame
                    frame_data = buffer[:self.frame_size]
                    buffer = buffer[self.frame_size:]
                    
                    # Convert to numpy array and reshape
                    frame = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = frame.reshape((self.height, self.width, 3))
                    
                    self.frames_received += 1
                    
                    # Add to queue, drop old frames if queue is full
                    try:
                        self.frame_queue.put_nowait(frame.copy())
                    except:
                        # Queue full, remove old frame and add new one
                        try:
                            self.frame_queue.get_nowait()
                            self.frame_queue.put_nowait(frame.copy())
                        except:
                            pass
                    
                    if self.frames_received % 30 == 0:
                        print(f"Received {self.frames_received} frames")
                        
            except Exception as e:
                print(f"Error in frame reader: {e}")
                break
        
        print("Frame reader thread ended")
    
    def get_frame(self):
        """Get the latest frame from queue"""
        try:
            return self.frame_queue.get_nowait()
        except:
            return None
    
    def has_frames(self):
        """Check if frames are available"""
        return not self.frame_queue.empty()
    
    def stop(self):
        """Stop the stream"""
        print("Stopping stream...")
        self.running = False
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        print(f"Stream stopped. Total frames received: {self.frames_received}")

def main():
    """Main function to run the stream viewer"""
    streamer = GStreamerOpenCV()
    
    if not streamer.start_stream():
        print("Failed to start stream")
        return
    
    print("Stream started. Waiting for first frame...")
    print("Press 'q' to quit, 's' to save frame, 'f' for fullscreen, 'r' to show stats")
    
    # Wait for first frame
    first_frame_time = time.time()
    first_frame = None
    
    while first_frame is None and time.time() - first_frame_time < 10:
        first_frame = streamer.get_frame()
        if first_frame is None:
            time.sleep(0.1)
    
    if first_frame is None:
        print("No frames received after 10 seconds. Check your stream.")
        streamer.stop()
        return
    
    print(f"First frame received! Shape: {first_frame.shape}")
    
    # Main display loop
    frame_count = 0
    start_time = time.time()
    last_fps_time = start_time
    fps_counter = 0
    current_fps = 0
    fullscreen = False
    
    cv2.namedWindow('UDP Video Stream', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('UDP Video Stream', 960, 540)  # Half size for display
    
    while True:
        frame = streamer.get_frame()
        
        if frame is not None:
            frame_count += 1
            fps_counter += 1
            
            # Calculate FPS every second
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                current_fps = fps_counter / (current_time - last_fps_time)
                fps_counter = 0
                last_fps_time = current_time
            
            # Add overlay information
            overlay_frame = frame.copy()
            
            # FPS and frame info
            cv2.putText(overlay_frame, f"FPS: {current_fps:.1f}", (30, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(overlay_frame, f"Frame: {frame_count}", (30, 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(overlay_frame, f"Size: {frame.shape[1]}x{frame.shape[0]}", (30, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            
            # Display the frame
            cv2.imshow('UDP Video Stream', overlay_frame)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and frame is not None:
            # Save current frame
            filename = f"frame_{frame_count}_{int(time.time())}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Frame saved as {filename}")
        elif key == ord('f'):
            # Toggle fullscreen
            if fullscreen:
                cv2.setWindowProperty('UDP Video Stream', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                cv2.resizeWindow('UDP Video Stream', 960, 540)
                fullscreen = False
                print("Windowed mode")
            else:
                cv2.setWindowProperty('UDP Video Stream', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                fullscreen = True
                print("Fullscreen mode")
        elif key == ord('r'):
            # Print stats
            elapsed = time.time() - start_time
            avg_fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"Stats: {frame_count} frames in {elapsed:.1f}s, avg FPS: {avg_fps:.1f}")
        
        # Small delay to prevent excessive CPU usage
        time.sleep(0.001)
    
    # Cleanup
    streamer.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()