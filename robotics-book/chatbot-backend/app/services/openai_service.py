from openai import OpenAI
from typing import List, Dict
from app.config import settings


class OpenAIService:
    def __init__(self):
        self.client = OpenAI(api_key=settings.openai_api_key)
        self.embedding_model = settings.embedding_model
        self.chat_model = settings.chat_model

    def get_embedding(self, text: str) -> List[float]:
        """Get embedding for a text"""
        response = self.client.embeddings.create(
            model=self.embedding_model,
            input=text
        )
        return response.data[0].embedding

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Get embeddings for multiple texts"""
        response = self.client.embeddings.create(
            model=self.embedding_model,
            input=texts
        )
        return [data.embedding for data in response.data]

    def chat_completion(
        self,
        user_message: str,
        context: str,
        selected_text: str = None
    ) -> str:
        """Generate chat completion with RAG context"""

        if selected_text:
            system_message = f"""You are a helpful AI assistant for a Physical AI & Humanoid Robotics course.
The user has selected specific text from the course material and is asking a question about it.

Selected Text:
{selected_text}

Relevant Context from the Course:
{context}

Answer the user's question based on the selected text and the provided context. Be specific and reference the selected text when relevant."""
        else:
            system_message = f"""You are a helpful AI assistant for a Physical AI & Humanoid Robotics course.
You have access to the course material and should answer questions based on that content.

Relevant Context from the Course:
{context}

Answer the user's question using the provided context. If the answer isn't in the context, say so politely and offer to help with related topics that are covered in the course."""

        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

        response = self.client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        return response.choices[0].message.content

    def health_check(self) -> bool:
        """Check if OpenAI API is accessible"""
        try:
            self.client.models.list()
            return True
        except:
            return False
