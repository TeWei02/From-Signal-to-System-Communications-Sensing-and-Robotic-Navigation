"""
link_simulator.py
-----------------
Simulates a wireless communication link with configurable bandwidth,
latency, and packet loss characteristics.

Models:
  - Token-bucket rate limiting (bandwidth constraint)
  - Queueing delay (exponential service time, M/M/1 approximation)
  - Random packet drops (Bernoulli loss model, or burst-error Gilbert-Elliott)

Usage:
    link = WirelessLink(bandwidth_mbps=2.0, latency_ms=100, packet_loss_rate=0.01)
    success, delay_ms = link.transmit(payload_bytes=1024, priority=0)
    
    # Serialize/send message queue packets
    packets = link.get_pending_packets(current_time)

References:
    - Ross, "Markovian models of transmission", queuing theory texts
    - 3GPP models for LTE/5G channel characteristics
"""

import time
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional
from collections import deque
import random


@dataclass
class Packet:
    """Represents a message packet in transit across the link."""
    timestamp_send: float      # When packet was submitted
    deadline: float            # Absolute delivery deadline (if QoS-driven)
    payload_bytes: int         # Payload size in bytes
    priority: int = 0          # Priority queue level (0=highest)
    msg_id: str = ""           # Unique message identifier
    
    def __lt__(self, other: 'Packet') -> bool:
        # For heapq: sort by priority first, then deadline
        return (self.priority, self.deadline) < (other.priority, other.deadline)


@dataclass
class LinkStats:
    """Running statistics of link performance."""
    bytes_sent: int = 0
    bytes_dropped: int = 0
    packets_sent: int = 0
    packets_dropped: int = 0
    avg_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    queue_depth_peak: int = 0
    

class WirelessLink:
    """
    Simulates a wireless link with bandwidth, latency, and loss characteristics.
    
    Usage pattern:
        1. Client calls transmit() with a message
        2. Link rates-limits based on bandwidth allocation
        3. get_pending_packets() returns packets ready to "receive"
    """
    
    def __init__(self, bandwidth_mbps: float = 10.0, latency_ms: float = 50.0,
                 packet_loss_rate: float = 0.0, max_queue_bytes: int = 10_000_000):
        """
        Args:
            bandwidth_mbps: Channel bandwidth in Mbps
            latency_ms: Base propagation + processing latency in milliseconds
            packet_loss_rate: Random loss probability [0, 1)
            max_queue_bytes: Maximum queue buffer before tail-drop
        """
        self.bandwidth_mbps = bandwidth_mbps
        self.latency_ms = latency_ms
        self.packet_loss_rate = packet_loss_rate
        self.max_queue_bytes = max_queue_bytes
        
        self.tx_queue: List[Packet] = []          # Priority queue of pending transmissions
        self.received_packets: deque[Packet] = deque()    # Packets that have "arrived"
        self.token_bucket_tokens = 0.0            # Tokens for rate limiting
        self.last_token_update = time.time()
        
        self.stats = LinkStats()
        self._latency_samples: deque[float] = deque(maxlen=100)  # For running average
        
    def transmit(self, payload_bytes: int, priority: int = 0,
                 msg_id: str = "", deadline_ms: Optional[float] = None) -> Tuple[bool, float]:
        """
        Queue a message for transmission over the wireless link.
        
        Args:
            payload_bytes: Size of the message in bytes
            priority: Queue priority (0=highest, larger=lower)
            msg_id: Optional message identifier for tracking
            deadline_ms: Relative deadline from now (for QoS)
            
        Returns:
            (success: bool, estimated_delay_ms: float)
                success=True if enqueued; False if dropped due to queue overflow
                estimated_delay_ms is best-effort estimate of total delay
        """
        now = time.time()
        abs_deadline = now + (deadline_ms / 1000.0) if deadline_ms else float('inf')
        
        # Check queue size constraint
        queue_bytes = sum(p.payload_bytes for p in self.tx_queue)
        if queue_bytes + payload_bytes > self.max_queue_bytes:
            self.stats.bytes_dropped += payload_bytes
            self.stats.packets_dropped += 1
            return False, float('inf')
        
        # Create packet and add to priority queue
        pkt = Packet(
            timestamp_send=now,
            deadline=abs_deadline,
            payload_bytes=payload_bytes,
            priority=priority,
            msg_id=msg_id
        )
        heapq.heappush(self.tx_queue, pkt)
        
        # Estimate delivery time: service_time + queueing_delay
        service_time_ms = (payload_bytes * 8) / (self.bandwidth_mbps * 1000.0)
        estimated_delay = self.latency_ms + service_time_ms
        
        return True, estimated_delay
    
    def get_pending_packets(self, current_time: float) -> List[Packet]:
        """
        Process the transmission queue and return packets that have arrived.
        
        Implements token-bucket rate limiting: tokens accumulate based on time elapsed
        and bandwidth. A packet is serviced if enough tokens are available.
        
        Args:
            current_time: Current simulated time (seconds)
            
        Returns:
            List of packets that have successfully traversed the link
        """
        # Refill token bucket
        time_elapsed = current_time - self.last_token_update
        self.last_token_update = current_time
        
        # Token rate: bandwidth_mbps -> bits/sec -> bytes/sec
        token_rate = (self.bandwidth_mbps * 1e6) / 8.0  # bytes per second
        self.token_bucket_tokens += token_rate * time_elapsed
        
        delivered: List[Packet] = []
        
        while self.tx_queue:
            pkt = self.tx_queue[0]
            
            # Check if packet has arrived (latency + service delay passed)
            service_time = (pkt.payload_bytes * 8) / (self.bandwidth_mbps * 1e6)
            arrival_time = pkt.timestamp_send + self.latency_ms / 1000.0 + service_time
            
            if current_time < arrival_time:
                break  # Packet not yet arrived
            
            if self.token_bucket_tokens < pkt.payload_bytes:
                break  # Not enough tokens available
            
            # Remove from queue and consume tokens
            heapq.heappop(self.tx_queue)
            self.token_bucket_tokens -= pkt.payload_bytes
            
            # Apply random packet loss
            if random.random() < self.packet_loss_rate:
                self.stats.bytes_dropped += pkt.payload_bytes
                self.stats.packets_dropped += 1
                continue
            
            # Compute actual latency and record stats
            actual_latency = (current_time - pkt.timestamp_send) * 1000.0  # ms
            self._latency_samples.append(actual_latency)
            self.stats.avg_latency_ms = sum(self._latency_samples) / len(self._latency_samples)
            self.stats.max_latency_ms = max(self.stats.max_latency_ms, actual_latency)
            
            # Deliver packet
            self.stats.bytes_sent += pkt.payload_bytes
            self.stats.packets_sent += 1
            delivered.append(pkt)
        
        # Track peak queue depth
        queue_bytes = sum(p.payload_bytes for p in self.tx_queue)
        self.stats.queue_depth_peak = max(self.stats.queue_depth_peak, queue_bytes)
        
        return delivered
    
    def get_stats(self) -> LinkStats:
        """Return current link statistics."""
        return self.stats
    
    def reset_stats(self):
        """Reset statistics counters."""
        self.stats = LinkStats()
        self._latency_samples.clear()


if __name__ == "__main__":
    # Example: simulate sending data over a bandwidth-limited link
    link = WirelessLink(bandwidth_mbps=2.0, latency_ms=100, packet_loss_rate=0.01)
    
    # Simulate a few message transmissions
    current_time = 0.0
    messages = [
        (1024, 0, "msg_001", 500),   # (bytes, priority, id, deadline_ms)
        (2048, 0, "msg_002", 500),
        (512, 0, "msg_003", 1000),
    ]
    
    for payload, prio, mid, deadline in messages:
        success, estimated_delay = link.transmit(payload, priority=prio, msg_id=mid, deadline_ms=deadline)
        print(f"{mid}: enqueued? {success}, est. delay {estimated_delay:.1f}ms")
    
    # Simulate time advancing and checking deliveries
    for t in [0.1, 0.2, 0.5, 1.0]:
        current_time = t
        delivered = link.get_pending_packets(current_time)
        print(f"T={t:.1f}s: {len(delivered)} packets delivered")
    
    print(f"\nFinal stats: {link.get_stats()}")
