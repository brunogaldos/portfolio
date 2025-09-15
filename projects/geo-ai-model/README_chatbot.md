# RAG Chatbot with Dynamic Document Upload

A modern web application that implements a Retrieval-Augmented Generation (RAG) chatbot with dynamic document upload capabilities. Users can upload their own .txt files and chat with an AI that answers questions based on the uploaded content.

## Features

- **Dynamic Document Upload**: Upload multiple .txt files through a modern drag-and-drop interface
- **RAG Implementation**: Uses Facebook's RAG model with DPR (Dense Passage Retrieval) for accurate document retrieval
- **Real-time Chat Interface**: Modern chat UI with real-time responses
- **Responsive Design**: Works seamlessly on desktop and mobile devices
- **File Management**: View, remove, and manage uploaded files before processing
- **Model Status Monitoring**: Real-time status indicators for model initialization

## Technology Stack

- **Backend**: Flask (Python)
- **Frontend**: HTML5, CSS3, JavaScript (Vanilla)
- **AI/ML**: 
  - Transformers (Hugging Face)
  - Sentence Transformers (DPR encoder)
  - BGE Reranker v2 M3 for document reranking
  - BART Large CNN for response generation
  - Hybrid search (FAISS + BM25/TF-IDF)
  - FAISS for vector similarity search
- **Styling**: Custom CSS with Font Awesome icons

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd RAG_CHATBOT
   ```

2. **Create a virtual environment** (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. **Start the application**:
   ```bash
   python app.py
   ```

2. **Open your browser** and navigate to:
   ```
   http://localhost:5000
   ```

3. **Upload Documents**:
   - Drag and drop .txt files or click to browse
   - Select multiple files if needed
   - Click "Upload and Initialize Model" to process the documents

4. **Start Chatting**:
   - Once the model is initialized, the chat interface will appear
   - Ask questions about the content in your uploaded documents
   - The AI will retrieve relevant information and generate responses

## How It Works

### RAG Architecture
1. **Document Processing**: Uploaded .txt files are chunked into smaller segments using semantic chunking
2. **Hybrid Search**: Combines FAISS vector similarity search with BM25/TF-IDF for better retrieval
3. **Document Reranking**: Uses BGE Reranker v2 M3 to rerank retrieved documents for relevance
4. **Response Generation**: BART Large CNN generates contextual summaries based on reranked passages
5. **Multi-Model Pipeline**: Sentence Transformer → Hybrid Search → Reranker → Summarizer

### File Structure
```
RAG_CHATBOT/
├── app.py                 # Flask backend application
├── requirements.txt       # Python dependencies
├── README.md             # This file
├── templates/
│   └── index.html        # Main HTML template
├── static/
│   ├── css/
│   │   └── style.css     # Custom styles
│   └── js/
│       └── app.js        # Frontend JavaScript
├── uploads/              # Uploaded files directory (auto-created)
└── *.txt                 # Sample text files
```

## API Endpoints

- `GET /` - Main application page
- `POST /upload` - Upload .txt files and initialize RAG model
- `POST /chat` - Send questions and receive AI responses
- `GET /status` - Check model initialization status

## Configuration

### File Upload Settings
- **Maximum file size**: 16MB per file
- **Supported formats**: .txt files only
- **Multiple files**: Yes, unlimited number of .txt files

### Model Settings
- **Sentence Transformer**: `sentence-transformers/facebook-dpr-ctx_encoder-multiset-base`
- **Reranker**: `BAAI/bge-reranker-v2-m3`
- **Summarizer**: `facebook/bart-large-cnn`
- **Chunk Size**: 50 words per chunk (configurable in `app.py`)
- **Search Method**: Hybrid (FAISS + BM25/TF-IDF)

## Customization

### Changing Chunk Size
Edit the `max_words` parameter in the `semantic_chunking` function in `app.py`:
```python
def semantic_chunking(documents, max_words=50):  # Change 50 to desired size
```

### Using Different Models
Modify the model initialization in `app.py`:
```python
# Change these lines to use different models
sentence_model = SentenceTransformer('your-sentence-transformer-model')
reranker_model = AutoModelForSequenceClassification.from_pretrained('your-reranker-model')
generator = pipeline('summarization', model='your-summarizer-model')
```

## Troubleshooting

### Common Issues

1. **Model initialization fails**:
   - Ensure you have sufficient RAM (at least 4GB recommended)
   - Check that all dependencies are installed correctly
   - Verify that uploaded files are valid .txt files

2. **Slow response times**:
   - The first query may take longer as models are loaded into memory
   - Consider using GPU acceleration if available
   - Reduce the number of uploaded documents

3. **Memory issues**:
   - Reduce chunk size in semantic chunking
   - Process fewer documents at once
   - Use smaller RAG models

### Performance Tips

- **GPU Acceleration**: Install PyTorch with CUDA support for faster processing
- **Batch Processing**: Upload multiple smaller files instead of one large file
- **Regular Cleanup**: Remove old uploaded files from the `uploads/` directory

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is open source and available under the MIT License.

## Acknowledgments

- Facebook Research for the RAG model implementation
- Hugging Face for the Transformers library
- FAISS for efficient similarity search
- The open-source community for various tools and libraries used in this project 