"""
Metrics and monitoring utilities for the RAG chatbot system
"""
import time
import asyncio
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum
from collections import defaultdict, deque
import threading
from datetime import datetime, timedelta


class MetricType(Enum):
    COUNTER = "counter"
    GAUGE = "gauge"
    HISTOGRAM = "histogram"
    SUMMARY = "summary"


@dataclass
class Metric:
    name: str
    type: MetricType
    value: float
    labels: Dict[str, str]
    timestamp: float


class MetricsRegistry:
    """
    Thread-safe metrics registry for collecting and storing metrics
    """
    def __init__(self):
        self._metrics: Dict[str, Metric] = {}
        self._counters: Dict[str, float] = defaultdict(float)
        self._gauges: Dict[str, float] = {}
        self._histograms: Dict[str, Dict[str, Any]] = defaultdict(lambda: {
            'values': [],
            'count': 0,
            'sum': 0.0,
            'buckets': defaultdict(int)
        })
        self._mutex = threading.Lock()
        self._request_times = deque(maxlen=1000)  # Keep last 1000 request times

    def increment_counter(self, name: str, labels: Dict[str, str] = None, amount: float = 1.0):
        """Increment a counter metric"""
        if labels is None:
            labels = {}

        key = f"{name}_{str(sorted(labels.items()))}"

        with self._mutex:
            self._counters[key] += amount

    def set_gauge(self, name: str, value: float, labels: Dict[str, str] = None):
        """Set a gauge metric"""
        if labels is None:
            labels = {}

        key = f"{name}_{str(sorted(labels.items()))}"

        with self._mutex:
            self._gauges[key] = value

    def observe_histogram(self, name: str, value: float, labels: Dict[str, str] = None):
        """Observe a value for a histogram metric"""
        if labels is None:
            labels = {}

        key = f"{name}_{str(sorted(labels.items()))}"

        with self._mutex:
            hist = self._histograms[key]
            hist['values'].append(value)
            hist['count'] += 1
            hist['sum'] += value

            # Simple bucketing for response times
            if name == 'response_time_seconds':
                if value < 0.1:
                    hist['buckets']['0.1'] += 1
                elif value < 0.5:
                    hist['buckets']['0.5'] += 1
                elif value < 1.0:
                    hist['buckets']['1.0'] += 1
                elif value < 2.0:
                    hist['buckets']['2.0'] += 1
                else:
                    hist['buckets']['+Inf'] += 1

    def record_request_time(self, duration: float):
        """Record a request processing time"""
        with self._mutex:
            self._request_times.append(duration)

    def get_request_stats(self) -> Dict[str, float]:
        """Get request time statistics"""
        with self._mutex:
            if not self._request_times:
                return {
                    'count': 0,
                    'avg': 0.0,
                    'p50': 0.0,
                    'p90': 0.0,
                    'p95': 0.0,
                    'p99': 0.0,
                    'min': 0.0,
                    'max': 0.0
                }

            sorted_times = sorted(self._request_times)
            count = len(sorted_times)

            return {
                'count': count,
                'avg': sum(sorted_times) / count,
                'p50': sorted_times[int(0.5 * count)],
                'p90': sorted_times[int(0.9 * count)],
                'p95': sorted_times[int(0.95 * count)],
                'p99': sorted_times[int(0.99 * count)],
                'min': sorted_times[0],
                'max': sorted_times[-1]
            }

    def get_counter(self, name: str, labels: Dict[str, str] = None) -> float:
        """Get current value of a counter"""
        if labels is None:
            labels = {}

        key = f"{name}_{str(sorted(labels.items()))}"
        return self._counters.get(key, 0.0)

    def get_gauge(self, name: str, labels: Dict[str, str] = None) -> float:
        """Get current value of a gauge"""
        if labels is None:
            labels = {}

        key = f"{name}_{str(sorted(labels.items()))}"
        return self._gauges.get(key, 0.0)

    def get_histogram(self, name: str, labels: Dict[str, str] = None) -> Dict[str, Any]:
        """Get histogram data"""
        if labels is None:
            labels = {}

        key = f"{name}_{str(sorted(labels.items()))}"
        return self._histograms.get(key, {'values': [], 'count': 0, 'sum': 0.0, 'buckets': {}})

    def get_all_metrics(self) -> Dict[str, Any]:
        """Get all collected metrics"""
        with self._mutex:
            return {
                'counters': dict(self._counters),
                'gauges': dict(self._gauges),
                'histograms': dict(self._histograms),
                'request_stats': self.get_request_stats()
            }


class MetricsMiddleware:
    """
    Middleware for collecting request metrics
    """
    def __init__(self, registry: MetricsRegistry):
        self.registry = registry

    async def __call__(self, call_next: Callable, request):
        """Middleware function to collect metrics for each request"""
        start_time = time.time()

        try:
            response = await call_next(request)
            status_code = response.status_code
        except Exception as e:
            status_code = 500
            raise
        finally:
            duration = time.time() - start_time

            # Record metrics
            self.registry.increment_counter('requests_total', {
                'method': request.method,
                'endpoint': request.url.path,
                'status_code': str(status_code)
            })

            self.registry.observe_histogram('response_time_seconds', duration, {
                'method': request.method,
                'endpoint': request.url.path
            })

            self.registry.record_request_time(duration)

        return response


class MetricsCollector:
    """
    Central metrics collection service for the RAG system
    """
    def __init__(self):
        self.registry = MetricsRegistry()
        self.middleware = MetricsMiddleware(self.registry)

    def start_timer(self) -> float:
        """Start a timer for measuring duration"""
        return time.time()

    def stop_timer(self, start_time: float) -> float:
        """Stop a timer and return the duration"""
        return time.time() - start_time

    def record_embedding_generation(self, duration: float, success: bool = True):
        """Record embedding generation metrics"""
        status = 'success' if success else 'error'

        self.registry.increment_counter('embedding_generation_total', {'status': status})
        self.registry.observe_histogram('embedding_generation_duration_seconds', duration, {'status': status})

    def record_qdrant_search(self, duration: float, results_count: int, success: bool = True):
        """Record Qdrant search metrics"""
        status = 'success' if success else 'error'

        self.registry.increment_counter('qdrant_search_total', {'status': status})
        self.registry.observe_histogram('qdrant_search_duration_seconds', duration, {'status': status})
        self.registry.set_gauge('qdrant_results_count', results_count)

    def record_llm_call(self, duration: float, tokens_used: int, success: bool = True):
        """Record LLM call metrics"""
        status = 'success' if success else 'error'

        self.registry.increment_counter('llm_calls_total', {'status': status})
        self.registry.observe_histogram('llm_call_duration_seconds', duration, {'status': status})
        self.registry.set_gauge('llm_tokens_used', tokens_used)

    def record_ingestion(self, duration: float, chunks_processed: int, success: bool = True):
        """Record document ingestion metrics"""
        status = 'success' if success else 'error'

        self.registry.increment_counter('ingestion_total', {'status': status})
        self.registry.observe_histogram('ingestion_duration_seconds', duration, {'status': status})
        self.registry.set_gauge('ingestion_chunks_processed', chunks_processed)

    def record_cache_hit(self, cache_type: str):
        """Record a cache hit"""
        self.registry.increment_counter('cache_hits_total', {'cache_type': cache_type})

    def record_cache_miss(self, cache_type: str):
        """Record a cache miss"""
        self.registry.increment_counter('cache_misses_total', {'cache_type': cache_type})

    def record_cache_size(self, cache_type: str, size: int):
        """Record cache size"""
        self.registry.set_gauge(f'{cache_type}_cache_size', size)

    def get_metrics_summary(self) -> Dict[str, Any]:
        """Get a summary of key metrics"""
        all_metrics = self.registry.get_all_metrics()
        request_stats = all_metrics['request_stats']

        return {
            'timestamp': datetime.now().isoformat(),
            'requests': {
                'total': all_metrics['request_stats']['count'],
                'avg_response_time': all_metrics['request_stats']['avg'],
                'p95_response_time': all_metrics['request_stats']['p95'],
                'p99_response_time': all_metrics['request_stats']['p99'],
            },
            'ingestion': {
                'total_calls': self.registry.get_counter('ingestion_total'),
                'avg_duration': self.registry.get_gauge('ingestion_duration_seconds_sum', {}) / max(1, self.registry.get_counter('ingestion_total')),
            },
            'embedding_generation': {
                'total_calls': self.registry.get_counter('embedding_generation_total'),
                'avg_duration': self.registry.get_gauge('embedding_generation_duration_seconds_sum', {}) / max(1, self.registry.get_counter('embedding_generation_total')),
            },
            'qdrant_search': {
                'total_calls': self.registry.get_counter('qdrant_search_total'),
                'avg_duration': self.registry.get_gauge('qdrant_search_duration_seconds_sum', {}) / max(1, self.registry.get_counter('qdrant_search_total')),
                'avg_results': self.registry.get_gauge('qdrant_results_count'),
            },
            'llm_calls': {
                'total_calls': self.registry.get_counter('llm_calls_total'),
                'avg_duration': self.registry.get_gauge('llm_call_duration_seconds_sum', {}) / max(1, self.registry.get_counter('llm_calls_total')),
            },
            'cache_performance': {
                'hits': self.registry.get_counter('cache_hits_total'),
                'misses': self.registry.get_counter('cache_misses_total'),
                'hit_rate': self.registry.get_counter('cache_hits_total') / max(1, self.registry.get_counter('cache_hits_total') + self.registry.get_counter('cache_misses_total')),
            }
        }

    async def export_prometheus_format(self) -> str:
        """Export metrics in Prometheus format"""
        all_metrics = self.registry.get_all_metrics()
        output = []

        # Add request metrics
        output.append("# HELP http_requests_total Total number of HTTP requests")
        output.append("# TYPE http_requests_total counter")
        for key, value in all_metrics['counters'].items():
            if key.startswith('requests_total'):
                # Extract labels from key
                parts = key.split('_')
                method = 'GET'  # Simplified for this example
                endpoint = 'unknown'
                status_code = '200'

                # Parse labels from key
                if 'method=' in key:
                    # This is a simplified version - in a real implementation you'd parse properly
                    pass

                output.append(f'http_requests_total{{method="{method}",endpoint="{endpoint}",status_code="{status_code}"}} {value}')

        # Add response time histogram
        output.append("# HELP http_response_time_seconds HTTP response time in seconds")
        output.append("# TYPE http_response_time_seconds histogram")

        # Add gauge metrics
        output.append("# HELP cache_hits_total Total cache hits")
        output.append("# TYPE cache_hits_total counter")
        output.append(f'cache_hits_total {all_metrics["counters"].get("cache_hits_total", 0)}')

        output.append("# HELP cache_misses_total Total cache misses")
        output.append("# TYPE cache_misses_total counter")
        output.append(f'cache_misses_total {all_metrics["counters"].get("cache_misses_total", 0)}')

        return '\n'.join(output)


# Global metrics collector instance
metrics_collector = MetricsCollector()


def get_metrics_collector() -> MetricsCollector:
    """Get the global metrics collector instance"""
    return metrics_collector


def track_function_metrics(func_name: str = None):
    """
    Decorator to track function execution metrics
    """
    def decorator(func):
        async def wrapper(*args, **kwargs):
            start_time = time.time()
            func_name_actual = func_name or func.__name__

            try:
                result = await func(*args, **kwargs)
                duration = time.time() - start_time

                # Record the successful execution
                metrics_collector.registry.increment_counter(
                    f'{func_name_actual}_executions_total',
                    {'status': 'success'}
                )
                metrics_collector.registry.observe_histogram(
                    f'{func_name_actual}_execution_duration_seconds',
                    duration,
                    {'status': 'success'}
                )

                return result
            except Exception as e:
                duration = time.time() - start_time

                # Record the failed execution
                metrics_collector.registry.increment_counter(
                    f'{func_name_actual}_executions_total',
                    {'status': 'error'}
                )
                metrics_collector.registry.observe_histogram(
                    f'{func_name_actual}_execution_duration_seconds',
                    duration,
                    {'status': 'error'}
                )

                raise
        return wrapper
    return decorator


# Initialize metrics collector
async def init_metrics():
    """
    Initialize the metrics system
    """
    # Currently no initialization needed, but kept for future use
    pass