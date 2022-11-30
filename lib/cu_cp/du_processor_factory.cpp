/*
 *
 * Copyright 2013-2022 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#include "srsgnb/cu_cp/du_processor_factory.h"
#include "du_processor_impl.h"

/// Notice this would be the only place were we include concrete class implementation files.

using namespace srsgnb;
using namespace srs_cu_cp;

std::unique_ptr<du_processor_interface>
srsgnb::srs_cu_cp::create_du_processor(const du_processor_config_t     du_processor_config_,
                                       du_processor_cu_cp_notifier&    cu_cp_notifier_,
                                       f1c_du_management_notifier&     f1c_du_mgmt_notifier_,
                                       f1c_message_notifier&           f1c_notifier_,
                                       rrc_ue_nas_notifier&            rrc_ue_nas_pdu_notifier_,
                                       rrc_ue_control_notifier&        rrc_ue_ngc_ctrl_notifier_,
                                       du_processor_ue_task_scheduler& task_sched_,
                                       du_processor_ue_manager&        ue_manager_)
{
  auto du_processor = std::make_unique<du_processor_impl>(du_processor_config_,
                                                          cu_cp_notifier_,
                                                          f1c_du_mgmt_notifier_,
                                                          f1c_notifier_,
                                                          rrc_ue_nas_pdu_notifier_,
                                                          rrc_ue_ngc_ctrl_notifier_,
                                                          task_sched_,
                                                          ue_manager_);
  return du_processor;
}
