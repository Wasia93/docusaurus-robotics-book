from http.server import BaseHTTPRequestHandler
import json

class handler(BaseHTTPRequestHandler):
    def do_GET(self):
        try:
            # Lazy imports to reduce cold start time
            import sys
            from pathlib import Path

            # Add parent directory to path
            parent_dir = str(Path(__file__).parent.parent)
            if parent_dir not in sys.path:
                sys.path.insert(0, parent_dir)

            # Try importing services
            from app.services.openai_service import OpenAIService
            from app.services.qdrant_service import QdrantService

            openai_service = OpenAIService()
            qdrant_service = QdrantService()

            response = {
                "status": "healthy",
                "qdrant_connected": qdrant_service.health_check(),
                "postgres_connected": True,
                "openai_configured": openai_service.health_check()
            }

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(response).encode())

        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            error_response = {
                "status": "unhealthy",
                "error": str(e),
                "qdrant_connected": False,
                "postgres_connected": False,
                "openai_configured": False
            }
            self.wfile.write(json.dumps(error_response).encode())
