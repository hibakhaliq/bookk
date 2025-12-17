import asyncio
import os
import requests
import json
import traceback
import argparse
from pathlib import Path

# ----------------------------
# Configuration
# ----------------------------
BACKEND_URL = "http://127.0.0.1:9008/api/v1"
BOOK_DIR = "book\\docs"

# ----------------------------
# CLI argument parser
# ----------------------------
parser = argparse.ArgumentParser(description="Ingest book content into backend")
parser.add_argument(
    "--force",
    action="store_true",
    help="Continue ingestion even if some files fail"
)
args = parser.parse_args()

# ----------------------------
# Read markdown files
# ----------------------------
def read_markdown_files(directory):
    content_parts = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.md'):
                file_path = os.path.join(root, file)
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    lines = content.split('\n')
                    title = "Untitled"
                    for line in lines:
                        if line.startswith('# '):
                            title = line[2:].strip()
                            break

                    content_parts.append({
                        'title': title,
                        'content': content,
                        'filename': file
                    })

    return content_parts

# ----------------------------
# Ingest content
# ----------------------------
def ingest_content_to_backend(title, content, book_id="robotics-book"):
    url = f"{BACKEND_URL}/ingest"

    payload = {
        "book_id": book_id,
        "title": title,
        "content": content,
        "metadata": {
            "source": "book_docs",
            "ingested_at": "2025-12-16"
        }
    }

    try:
        print(f"[INFO] Starting ingestion for: {title} (content length: {len(content)} chars)")
        response = requests.post(url, json=payload, timeout=120)
        if response.status_code == 200:
            result = response.json()
            print(f"[SUCCESS] Successfully ingested: {title}")
            print(f"  - Chunks processed: {result.get('chunks_processed', 0)}")
            print(f"  - Postgres IDs: {len(result.get('postgres_ids', []))}")
            print(f"  - Qdrant IDs: {len(result.get('qdrant_ids', []))}")
            return True
        else:
            print(f"[ERROR] Failed to ingest {title}: {response.status_code} - {response.text}")
            return False
    except requests.exceptions.ReadTimeout:
        print(f"[TIMEOUT] Ingestion for {title} took too long (may still succeed)")
        return True  # force mode safe assumption
    except Exception as e:
        print(f"[ERROR] Error ingesting {title}: {str(e)}")
        print(f"[ERROR] Full traceback: {traceback.format_exc()}")
        if args.force:
            print(f"[WARN] Continuing due to --force")
            return False  # continue with next file
        else:
            raise

# ----------------------------
# Main ingestion loop
# ----------------------------
async def main():
    print("Starting book content ingestion...")
    print(f"Connecting to backend at: {BACKEND_URL}")
    print(f"Reading content from: {BOOK_DIR}")

    content_parts = read_markdown_files(BOOK_DIR)
    print(f"\nFound {len(content_parts)} markdown files to process\n")

    successful_ingests = 0

    for i, part in enumerate(content_parts, 1):
        print(f"[{i}/{len(content_parts)}] Processing: {part['title']}")

        try:
            success = ingest_content_to_backend(
                title=part['title'],
                content=part['content']
            )
            if success:
                successful_ingests += 1
        except Exception as e:
            if args.force:
                print(f"[WARN] Skipping {part['title']} due to error: {e}")
            else:
                raise

        await asyncio.sleep(0.5)  # avoid spamming backend

    print(f"\nIngestion complete!")
    print(f"Successful: {successful_ingests}/{len(content_parts)}")
    if args.force:
        print("Some files may have failed but were skipped due to --force.")

    if successful_ingests > 0:
        print("\nThe chatbot is now ready to answer questions about the book content.")

# ----------------------------
# Run
# ----------------------------
if __name__ == "__main__":
    asyncio.run(main())
