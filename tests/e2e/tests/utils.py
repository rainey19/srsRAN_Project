#
# Copyright 2021-2023 Software Radio Systems Limited
#
# By using this file, you agree to the terms and conditions set
# forth in the LICENSE file which can be found at the top level of
# the distribution.
#

import logging
from collections import defaultdict
from contextlib import contextmanager, suppress
from pprint import pformat

import grpc
from pytest_html import extras
from retina.launcher import data
from retina.protocol.base_pb2 import Empty

ATTACH_TIMEOUT = 120
STARTUP_TIMEOUT = 3 * 60  # RF requires more time
DEFAULT_MCS = 10


@contextmanager
def get_ue_gnb_epc(
    self,
    extra,
    band,
    common_scs,
    bandwidth,
    mcs,
    ue_count,
    log_search,
    global_timing_advance,
    time_alignment_calibration,
):
    """
    Get test elements
    """
    test_successful = True
    try:
        test_config = {
            "ue": {
                "parameters": {
                    "band": band,
                    "dl_arfcn": get_dl_arfcn(band),
                    "ssb_arfcn": get_ssb_arfcn(band, bandwidth),
                    "common_scs": common_scs,
                    "ssb_scs": common_scs,
                    "bandwidth": bandwidth,
                    "ue_count": ue_count,
                    "global_timing_advance": global_timing_advance,
                },
            },
            "gnb": {
                "parameters": {
                    "band": band,
                    "dl_arfcn": get_dl_arfcn(band),
                    "common_scs": common_scs,
                    "bandwidth": bandwidth,
                    "mcs": mcs,
                    "time_alignment_calibration": time_alignment_calibration,
                },
            },
        }
        if is_tdd(band):
            test_config["ue"]["parameters"]["rx_ant"] = "rx"

        logging.info("Test config: \n%s", pformat(test_config))
        self.test_config = test_config
        self.retina_manager.parse_configuration(test_config)

        # Get clients
        ue = self.retina_manager.get_ue()
        gnb = self.retina_manager.get_gnb()
        epc = self.retina_manager.get_epc()

        yield ue, gnb, epc

    except Exception as err:
        test_successful = False
        raise err from None

    finally:
        teardown_ok = True
        log_search_ok = True

        with suppress(NameError, grpc._channel._InactiveRpcError):
            return_code = gnb.Stop(Empty()).value
            if return_code < 0:
                teardown_ok = False
                logging.error("GNB crashed with exit code %s", return_code)
            elif return_code > 0:
                if log_search:
                    log_search_ok = False
                logging.error("GNB has %d errors or warnings", return_code)
        with suppress(NameError, grpc._channel._InactiveRpcError):
            epc.Stop(Empty()).value
        with suppress(NameError, grpc._channel._InactiveRpcError):
            return_code = ue.Stop(Empty()).value
            teardown_ok &= return_code == 0
            if return_code < 0:
                teardown_ok = False
                logging.error("UE crashed with exit code %s", return_code)
            elif return_code > 0:
                if log_search:
                    log_search_ok = False
                logging.error("UE has %d errors or warnings", return_code)
        if not teardown_ok or not log_search_ok:
            test_successful = False

        if test_successful:
            self.disable_download_report()
        if test_successful is False or data.get_force_download():
            with suppress(UnboundLocalError, NameError):
                extra.append(extras.url(self.relative_output_html_path, name="[[ Go to logs and configs ]]"))

        assert teardown_ok is True, "GNB or UE crashed!"
        assert log_search_ok is True, "There are errors in the log!"


def is_tdd(band):
    return band in (41, 78)


def get_dl_arfcn(band):
    """
    Get dl arfcn
    """
    return {3: 368500, 7: 536020, 41: 520002}[band]


def get_ssb_arfcn(band, bandwidth):
    """
    Get SSB arfcn
    """
    return {
        3: defaultdict(
            lambda: 368410,
            {
                30: 367450,
                40: 366490,
                50: 365530,
            },
        ),
        7: defaultdict(
            lambda: 535930,
            {
                30: 534970,
                40: 534010,
                50: 533050,
            },
        ),
        41: defaultdict(
            lambda: 519870,
            {
                20: 520110,
                30: 518910,
                40: 517950,
                50: 517230,
            },
        ),
    }[band][bandwidth]
