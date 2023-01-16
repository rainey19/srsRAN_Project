/*
 *
 * Copyright 2013-2022 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */
#include "gtpu_pdu.h"
#include "srsgnb/support/bit_encoding.h"

namespace srsgnb {

bool gtpu_read_ext_header(bit_decoder&                            decoder,
                          uint8_t                                 header_type,
                          std::unique_ptr<gtpu_extension_header>& ext,
                          srslog::basic_logger&                   logger);

bool gtpu_read_ext_pdu_session_container(bit_decoder&                            decoder,
                                         std::unique_ptr<gtpu_extension_header>& ext,
                                         srslog::basic_logger&                   logger);

bool gtpu_write_ext_header(bit_encoder&                                  encoder,
                           const std::unique_ptr<gtpu_extension_header>& ext,
                           uint8_t                                       next_extension_header_type,
                           srslog::basic_logger&                         logger);

bool gtpu_write_ext_pdu_session_container(bit_encoder&                                  decoder,
                                          const std::unique_ptr<gtpu_extension_header>& ext,
                                          uint8_t                                       next_extension_header_type,
                                          srslog::basic_logger&                         logger);

/****************************************************************************
 * Header pack/unpack helper functions
 * Ref: 3GPP TS 29.281 v10.1.0 Section 5
 ***************************************************************************/
bool gtpu_write_header(byte_buffer& pdu, const gtpu_header& header, srslog::basic_logger& logger)
{
  // flags
  if (!gtpu_supported_flags_check(header, logger)) {
    logger.error("gtpu_write_header - Unhandled GTP-U Flags. Flags: {}", header.flags);
    return false;
  }

  // msg type
  if (!gtpu_supported_msg_type_check(header, logger)) {
    logger.error("gtpu_write_header - Unhandled GTP-U Message Type. Message Type: {}", header.message_type);
    return false;
  }

  byte_buffer hdr_buf;
  bit_encoder encoder{hdr_buf};

  // Flags
  encoder.pack(header.flags.version, 3);
  encoder.pack(header.flags.protocol_type, 1);
  encoder.pack(0, 1);                               // Reserved
  encoder.pack(header.flags.ext_hdr ? 1 : 0, 1);    // E
  encoder.pack(header.flags.seq_number ? 1 : 0, 1); // S
  encoder.pack(header.flags.n_pdu ? 1 : 0, 1);      // PN

  // Message type
  encoder.pack(header.message_type, 8);

  // Length
  encoder.pack(header.length, 16);

  // TEID
  encoder.pack(header.teid, 32);

  // Optional header fields
  if (header.flags.ext_hdr || header.flags.seq_number || header.flags.n_pdu) {
    // Sequence Number
    encoder.pack(header.seq_number, 16);

    // N-PDU
    encoder.pack(header.n_pdu, 8);

    // Next Extension Header Type
    encoder.pack(header.next_ext_hdr_type, 8);
  }

  // TODO write header extensions
  for (uint16_t i = 0; i < header.ext_list.size(); ++i) {
    if (i == (header.ext_list.size() - 1)) {
      gtpu_write_ext_header(encoder, header.ext_list[i], GTPU_EXT_NO_MORE_EXTENSION_HEADERS, logger);
    } else {
      gtpu_write_ext_header(encoder, header.ext_list[i], header.ext_list[i + 1]->extension_header_type, logger);
    }
  }

  pdu.chain_before(std::move(hdr_buf));
  return true;
}

bool gtpu_read_teid(uint32_t& teid, const byte_buffer& pdu, srslog::basic_logger& logger)
{
  if (pdu.length() < GTPU_BASE_HEADER_LEN) {
    logger.error(pdu.begin(), pdu.end(), "Error GTP-U PDU is too small. Length={}", pdu.length());
    return false;
  }
  teid                          = {};
  byte_buffer_reader pdu_reader = pdu;
  pdu_reader += 4;
  for (int i = 3; i >= 0; --i) {
    teid |= (*pdu_reader << (i * 8U));
    ++pdu_reader;
  }
  return true;
}

bool gtpu_read_and_strip_header(gtpu_header& header, byte_buffer& pdu, srslog::basic_logger& logger)
{
  if (pdu.length() < GTPU_BASE_HEADER_LEN) {
    logger.error(pdu.begin(), pdu.end(), "Error GTP-U PDU is too small. Length={}", pdu.length());
    return false;
  }

  bit_decoder decoder{pdu};

  // Flags
  decoder.unpack(header.flags.version, 3);
  decoder.unpack(header.flags.protocol_type, 1);
  uint8_t spare = {};
  decoder.unpack(spare, 1);                   // Reserved
  decoder.unpack(header.flags.ext_hdr, 1);    // E
  decoder.unpack(header.flags.seq_number, 1); // S
  decoder.unpack(header.flags.n_pdu, 1);      // PN

  // Check supported flags
  if (!gtpu_supported_flags_check(header, logger)) {
    logger.error("gtpu_read_header - Unhandled GTP-U Flags. Flags: {}", header.flags);
    return false;
  }

  // Message type
  decoder.unpack(header.message_type, 8);

  // Length
  decoder.unpack(header.length, 16);

  // TEID
  decoder.unpack(header.teid, 32);

  // Optional header fields
  if (header.flags.ext_hdr || header.flags.seq_number || header.flags.n_pdu) {
    // Sanity check PDU length
    if (pdu.length() < GTPU_EXTENDED_HEADER_LEN) {
      logger.error(pdu.begin(), pdu.end(), "Error extended GTP-U PDU is too small. Length={}", pdu.length());
      return false;
    }

    // Sequence Number
    decoder.unpack(header.seq_number, 16);

    // N-PDU
    decoder.unpack(header.n_pdu, 8);

    // Next Extension Header Type
    decoder.unpack(header.next_ext_hdr_type, 8);
  }

  // Read Header Extensions
  if (header.flags.ext_hdr) {
    if (header.next_ext_hdr_type == GTPU_EXT_NO_MORE_EXTENSION_HEADERS) {
      logger.error(pdu.begin(), pdu.end(), "Error E flag is set, but there are no extra extensions");
      return false;
    }
    std::unique_ptr<gtpu_extension_header> ext = nullptr;
    if (!gtpu_read_ext_header(decoder, header.next_ext_hdr_type, ext, logger)) {
      return false;
    }
    header.ext_list.push_back(std::move(ext));
    // if (ext->next_header_type == GTPU_EXT_NO_MORE_EXTENSION_HEADERS)
  }
  // Trim header
  pdu.trim_head(decoder.nof_bytes());

  return true;
}

bool gtpu_read_ext_header(bit_decoder&                            decoder,
                          uint8_t                                 header_type,
                          std::unique_ptr<gtpu_extension_header>& ext,
                          srslog::basic_logger&                   logger)
{
  switch (header_type) {
    case GTPU_EXT_NO_MORE_EXTENSION_HEADERS:
      logger.error("Called for header extension unpacking, but there is no extension to unpack");
      return false;
    case GTPU_EXT_RESERVED_0:
    case GTPU_EXT_RESERVED_1:
    case GTPU_EXT_RESERVED_2:
    case GTPU_EXT_RESERVED_3:
      logger.error("Header extension type is reserved");
      return false;
    case GTPU_EXT_HEADER_PDU_SESSION_CONTAINER:
      return gtpu_read_ext_pdu_session_container(decoder, ext, logger);
    default:
      logger.error("Unhandled header extension");
      return false;
  }

  return true;
}

bool gtpu_write_ext_header(bit_encoder&                                  encoder,
                           const std::unique_ptr<gtpu_extension_header>& ext,
                           uint8_t                                       next_extension_header_type,
                           srslog::basic_logger&                         logger)
{
  switch (ext->extension_header_type) {
    case GTPU_EXT_NO_MORE_EXTENSION_HEADERS:
      logger.error("Called for header extension packing, but there is no extension to unpack");
      return false;
    case GTPU_EXT_RESERVED_0:
    case GTPU_EXT_RESERVED_1:
    case GTPU_EXT_RESERVED_2:
    case GTPU_EXT_RESERVED_3:
      logger.error("Header extension type is reserved");
      return false;
    case GTPU_EXT_HEADER_PDU_SESSION_CONTAINER:
      return gtpu_write_ext_pdu_session_container(encoder, ext, next_extension_header_type, logger);
    default:
      logger.error("Unhandled header extension packing");
      return false;
  }

  return true;
}

bool gtpu_read_ext_pdu_session_container(bit_decoder&                            decoder,
                                         std::unique_ptr<gtpu_extension_header>& ext,
                                         srslog::basic_logger&                   logger)
{
  auto ext_ptr = std::make_unique<gtpu_extension_header_pdu_session_container>();
  decoder.unpack(ext_ptr->length, 8);

  // The payload size is four bytes per the indicated length,
  // minus one byte for the length field and one for the next
  // extension header type. See section 5.2.1 of 29.281.
  uint16_t payload = ext_ptr->length * 4 - 2;

  // TODO check max size

  // Extract container
  ext_ptr->container.resize(payload);
  for (unsigned i = 0; i < ext_ptr->container.size(); ++i) {
    decoder.unpack(ext_ptr->container[i], 8);
  }

  // Extract next extension header type
  decoder.unpack(ext_ptr->next_extension_header_type, 8);

  ext                        = std::move(ext_ptr);
  ext->extension_header_type = GTPU_EXT_HEADER_PDU_SESSION_CONTAINER;
  return true;
}

bool gtpu_write_ext_pdu_session_container(bit_encoder&                                  encoder,
                                          const std::unique_ptr<gtpu_extension_header>& ext,
                                          uint8_t                                       next_header_extension_type,
                                          srslog::basic_logger&                         logger)
{
  const gtpu_extension_header_pdu_session_container* ext_ptr =
      dynamic_cast<gtpu_extension_header_pdu_session_container*>(ext.get());

  uint8_t payload = 1 + ext_ptr->container.size() + 1;
  uint8_t length  = payload / 4;

  // Pack length
  encoder.pack(length, 8);

  // Pack container
  for (unsigned i = 0; i < ext_ptr->container.size(); ++i) {
    encoder.pack(ext_ptr->container[i], 8);
  }

  // Pack next header extension type
  encoder.pack(next_header_extension_type, 8);
  return true;
}
} // namespace srsgnb
