import math

def nmea_checksum_ok(sentence: str) -> bool:
    """Check if NMEA sentence has valid checksum."""
    try:
        body, chk = sentence.strip()[1:].split('*')
    except ValueError:
        return False
    calc = 0
    for c in body:
        calc ^= ord(c)
    try:
        return int(chk, 16) == calc
    except ValueError:
        return False
    
def dm_to_deg(dm: str, hemi: str) -> float:
    """Convert NMEA ddmm.mmmm (or dddmm.mmmm) to decimal degrees."""
    if not dm or '.' not in dm:
        return float('nan')
    dot = dm.find('.')
    head = dm[:dot]
    deg = int(head[:-2]) if len(head) > 2 else 0
    minutes = float(dm[dot-2:])
    val = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        val = -val
    return val

def knots_to_mps(k) -> float:
    """Convert knots to meters per second."""
    return float(k) * 0.514444 if k not in (None, '',) else 0.0