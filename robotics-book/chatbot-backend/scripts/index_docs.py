"""
Script to index all documentation files into the RAG system
"""
import os
import sys
from pathlib import Path
import markdown
from bs4 import BeautifulSoup

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from app.services.rag_service import RAGService
from app.services.qdrant_service import QdrantService


def extract_text_from_markdown(md_content: str) -> str:
    """Convert markdown to plain text"""
    html = markdown.markdown(md_content)
    soup = BeautifulSoup(html, 'html.parser')
    return soup.get_text()


def index_documentation():
    """Index all documentation files"""
    print("Initializing RAG service...")
    rag_service = RAGService()
    qdrant_service = QdrantService()

    # Initialize Qdrant collection
    print("Initializing Qdrant collection...")
    qdrant_service.initialize_collection()

    # Path to docs directory
    docs_dir = Path(__file__).parent.parent.parent / "docs"

    if not docs_dir.exists():
        print(f"Docs directory not found: {docs_dir}")
        return

    indexed_count = 0
    total_chunks = 0

    # Recursively index all markdown files
    for md_file in docs_dir.rglob("*.md"):
        try:
            # Read file
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract plain text
            text_content = extract_text_from_markdown(content)

            # Get relative path as doc_id
            doc_id = str(md_file.relative_to(docs_dir)).replace('\\', '/')

            # Get title (filename without extension)
            title = md_file.stem.replace('-', ' ').title()

            print(f"Indexing: {doc_id}")

            # Index document
            chunks = rag_service.index_document(
                doc_id=doc_id,
                title=title,
                content=text_content,
                metadata={
                    "file_path": str(md_file),
                    "category": md_file.parent.name
                }
            )

            indexed_count += 1
            total_chunks += chunks
            print(f"  ✓ Indexed {chunks} chunks")

        except Exception as e:
            print(f"  ✗ Error indexing {md_file}: {e}")

    print(f"\n{'='*50}")
    print(f"Indexing Complete!")
    print(f"Files indexed: {indexed_count}")
    print(f"Total chunks: {total_chunks}")
    print(f"{'='*50}")


if __name__ == "__main__":
    print("Starting documentation indexing...")
    index_documentation()
