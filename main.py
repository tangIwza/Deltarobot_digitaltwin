"""
Delta Robot Controller Launcher
===============================
Starts the FastAPI web server and launches the default web browser.
"""

import sys
import webbrowser
import threading
import time
import uvicorn
from server import app

def open_browser():
    time.sleep(1.2)
    webbrowser.open("http://localhost:8000")

if __name__ == "__main__":
    print("\n" + "="*65)
    print("  3D DELTA ROBOT CONTROLLER - STARTING APPLICATION")
    print("  Actuators: 3x NEMA 23 (57BYG250C)")
    print("  Bicep Length: 150 mm | Forearm Length: 400 mm")
    print("  Web Interface: http://localhost:8000")
    print("="*65 + "\n")
    
    # Auto-open browser
    threading.Thread(target=open_browser, daemon=True).start()
    
    uvicorn.run(app, host="127.0.0.1", port=8000)
