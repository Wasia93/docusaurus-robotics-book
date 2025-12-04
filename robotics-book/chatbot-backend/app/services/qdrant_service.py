from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict
from app.config import settings
import uuid


class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name

    def initialize_collection(self, vector_size: int = 1536):
        """Initialize Qdrant collection if it doesn't exist"""
        try:
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE
                    )
                )
                print(f"Created collection: {self.collection_name}")
            else:
                print(f"Collection already exists: {self.collection_name}")
        except Exception as e:
            print(f"Error initializing collection: {e}")
            raise

    def add_documents(self, embeddings: List[List[float]], metadata: List[Dict]):
        """Add document embeddings to Qdrant"""
        points = [
            PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload=meta
            )
            for embedding, meta in zip(embeddings, metadata)
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        return len(points)

    def search(self, query_embedding: List[float], limit: int = 5) -> List[Dict]:
        """Search for similar documents"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        return [
            {
                "content": hit.payload.get("content", ""),
                "title": hit.payload.get("title", ""),
                "doc_id": hit.payload.get("doc_id", ""),
                "score": hit.score
            }
            for hit in results
        ]

    def health_check(self) -> bool:
        """Check if Qdrant is accessible"""
        try:
            self.client.get_collections()
            return True
        except:
            return False
