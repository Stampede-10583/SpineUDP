from spine_interfaces.msg import LinkStats
from spine_stats.node import ORIN_COUNTERS, stats_to_dict


def test_every_linkstats_counter_is_published():
    counters = {f for f, t in LinkStats.get_fields_and_field_types().items() if t == 'uint64'}
    assert counters == set(ORIN_COUNTERS)


def test_stats_to_dict_uses_doc_names():
    msg = LinkStats()
    msg.poses_sent = 7
    msg.withheld_out_of_order = 2
    msg.last_restart_controller_timestamp = float('nan')
    d = stats_to_dict(msg)
    assert d['posesSent'] == 7
    assert d['withheldOutOfOrder'] == 2
    assert set(d) >= {'posesSent', 'withheldNoFix', 'withheldStale', 'withheldBadTimestamp',
                      'odomRecvd', 'odomSeqGaps', 'trigRecvd', 'triggersMissed',
                      'decodeErrors', 'sendErrors', 'rioRestarts'}
