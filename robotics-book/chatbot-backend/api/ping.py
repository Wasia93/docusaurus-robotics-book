from http.server import BaseHTTPRequestHandler
import json
import os

class handler(BaseHTTPRequestHandler):
    def do_GET(self):
        response = {
            "status": "ok",
            "message": "Vercel Python is working!",
            "env_check": {
                "openai_key_set": "OPENAI_API_KEY" in os.environ,
                "qdrant_url_set": "QDRANT_URL" in os.environ,
                "database_url_set": "DATABASE_URL" in os.environ
            }
        }

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(response, indent=2).encode())
