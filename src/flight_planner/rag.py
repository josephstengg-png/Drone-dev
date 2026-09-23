from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

TEMPLATE_DIR = Path(__file__).parent / "templates"
_TOKEN_RE = re.compile(r"[a-zA-Z0-9]+")


def _tokenize(text: str) -> list[str]:
    return _TOKEN_RE.findall(text.lower())


@dataclass(frozen=True)
class TemplateChunk:
    source: str
    heading: str
    text: str


def _split_sections(path: Path) -> list[TemplateChunk]:
    chunks: list[TemplateChunk] = []
    heading = path.stem
    buffer: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("#"):
            if buffer and "".join(buffer).strip():
                chunks.append(TemplateChunk(path.name, heading, "\n".join(buffer).strip()))
            heading = line.lstrip("#").strip()
            buffer = []
        else:
            buffer.append(line)
    if buffer and "".join(buffer).strip():
        chunks.append(TemplateChunk(path.name, heading, "\n".join(buffer).strip()))
    return chunks


def load_template_chunks(template_dir: Path = TEMPLATE_DIR) -> list[TemplateChunk]:
    chunks: list[TemplateChunk] = []
    for path in sorted(template_dir.glob("*.md")):
        chunks.extend(_split_sections(path))
    return chunks


class TemplateStore:
    """Offline TF-IDF retriever over the local QGC plan template corpus — the RAG index for Gemma prompts."""

    def __init__(self, chunks: list[TemplateChunk] | None = None):
        self.chunks = chunks if chunks is not None else load_template_chunks()
        doc_tokens = [_tokenize(chunk.text) for chunk in self.chunks]
        document_frequency: dict[str, int] = {}
        for tokens in doc_tokens:
            for term in set(tokens):
                document_frequency[term] = document_frequency.get(term, 0) + 1
        document_count = len(self.chunks)
        self._document_frequency = document_frequency
        self._document_count = document_count
        self._doc_vectors = [self._vectorize(tokens) for tokens in doc_tokens]

    def _idf(self, term: str) -> float:
        return math.log((self._document_count + 1) / (self._document_frequency.get(term, 0) + 1)) + 1.0

    def _vectorize(self, tokens: list[str]) -> dict[str, float]:
        counts: dict[str, int] = {}
        for term in tokens:
            counts[term] = counts.get(term, 0) + 1
        return {term: count * self._idf(term) for term, count in counts.items()}

    @staticmethod
    def _cosine(a: dict[str, float], b: dict[str, float]) -> float:
        common = set(a) & set(b)
        if not common:
            return 0.0
        numerator = sum(a[term] * b[term] for term in common)
        norm_a = math.sqrt(sum(value * value for value in a.values()))
        norm_b = math.sqrt(sum(value * value for value in b.values()))
        if norm_a == 0 or norm_b == 0:
            return 0.0
        return numerator / (norm_a * norm_b)

    def retrieve(self, query: str, top_k: int = 3) -> list[TemplateChunk]:
        if not self.chunks:
            return []
        query_vector = self._vectorize(_tokenize(query))
        scored = sorted(
            zip((self._cosine(query_vector, doc_vector) for doc_vector in self._doc_vectors), self.chunks),
            key=lambda pair: pair[0],
            reverse=True,
        )
        top = [chunk for score, chunk in scored[:top_k] if score > 0]
        return top or self.chunks[:top_k]
