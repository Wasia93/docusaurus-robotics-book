from fastapi import FastAPI
from mangum import Mangum

app = FastAPI()

@app.get("/")
def root():
    return {"message": "Vercel test working!"}

@app.get("/api/test")
def test():
    return {"status": "ok", "environment": "vercel"}

handler = Mangum(app, lifespan="off")
