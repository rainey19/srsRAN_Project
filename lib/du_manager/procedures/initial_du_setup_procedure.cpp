/*
 *
 * Copyright 2013-2022 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#include "initial_du_setup_procedure.h"
#include "../../f1_interface/converters/f1_procedure_helpers.h"
#include "../converters/mac_cell_configuration_helpers.h"
#include "../du_cell_manager.h"
#include "srsgnb/asn1/f1ap.h"

using namespace srsgnb;

initial_du_setup_procedure::initial_du_setup_procedure(const du_manager_config_t& cfg_, du_cell_manager& cell_mng_) :
  cfg(cfg_), cell_mng(cell_mng_)
{}

void initial_du_setup_procedure::operator()(coro_context<async_task<void> >& ctx)
{
  CORO_BEGIN(ctx);

  f1ap::fill_f1_setup_request_message(request_msg, cfg.setup_params);

  // Initiate F1 Setup.
  CORO_AWAIT_VALUE(response_msg, cfg.f1ap->handle_f1ap_setup_request(request_msg));

  // Handle F1 setup result.
  if (response_msg.success) {
    // In case of F1 Setup Response.
    handle_f1_setup_response(response_msg.msg);
  }

  // Configure DU Cells.
  for (unsigned idx = 0; idx < cell_mng.nof_cells(); ++idx) {
    du_cell_index_t cell_index = to_du_cell_index(idx);
    cfg.mac_cell_mng->add_cell(make_mac_cell_config(cell_index, cell_mng.get_cell_cfg(cell_index)));
  }

  // Activate DU Cells.
  cfg.mac_cell_mng->get_cell_controller(to_du_cell_index(0)).start();

  CORO_RETURN();
}

void initial_du_setup_procedure::handle_f1_setup_response(const asn1::f1ap::f1_setup_resp_s& resp)
{
  // TODO

  if (resp->cells_to_be_activ_list_present) {
    for (const auto& cell : resp->cells_to_be_activ_list.value) {
      const asn1::f1ap::cells_to_be_activ_list_item_s& item = cell->cells_to_be_activ_list_item();
      (void)item;
      // TODO: Handle cells to activate.
    }
  }
}
