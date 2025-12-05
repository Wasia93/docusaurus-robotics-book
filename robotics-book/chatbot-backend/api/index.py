import os
import sys
from pathlib import Path

# Add parent directory to path
parent_dir = str(Path(__file__).parent.parent)
sys.path.insert(0, parent_dir)

# Set working directory for imports
os.chdir(parent_dir)

from mangum import Mangum
from app.main import app

# Wrap FastAPI app with Mangum for serverless
handler = Mangum(app, lifespan="off")
