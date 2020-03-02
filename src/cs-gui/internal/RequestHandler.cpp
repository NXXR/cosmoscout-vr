////////////////////////////////////////////////////////////////////////////////////////////////////
//                               This file is part of CosmoScout VR                               //
//      and may be used under the terms of the MIT license. See the LICENSE file for details.     //
//                        Copyright: (c) 2019 German Aerospace Center (DLR)                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RequestHandler.hpp"
#include "../../cs-utils/utils.hpp"
#include "ResourceRequestHandler.hpp"

#include <fstream>
#include <include/wrapper/cef_stream_resource_handler.h>
#include <iostream>
#include <spdlog/spdlog.h>

namespace cs::gui::detail {

////////////////////////////////////////////////////////////////////////////////////////////////////

bool RequestHandler::OnCertificateError(CefRefPtr<CefBrowser> browser, cef_errorcode_t cert_error,
    CefString const& request_url, CefRefPtr<CefSSLInfo> ssl_info,
    CefRefPtr<CefRequestCallback> callback) {

  spdlog::warn("Detected a certificate error in Chromium Embedded Framework. Continuing...");

  callback->Continue(true);
  return true;
}
bool RequestHandler::OnBeforeBrowse(CefRefPtr<CefBrowser> browser, CefRefPtr<CefFrame> frame,
    CefRefPtr<CefRequest> request, bool user_gesture, bool is_redirect) {

  auto url = request->GetURL().ToString();

  // TODO it would be better to create a whitelist (or blacklist) of specific urls.
  if (utils::startsWith(url, "http") || utils::startsWith(url, "www")) {
    spdlog::info("Opening the following URL in an external browser: {}", url);

    // Open links in the default OS browser to avoid breaking the CosmoScout GUI.
    if constexpr (utils::HostOS == utils::OS::Linux) {
      std::string command = "xdg-open " + url + " >/dev/null 2>&1";
      system(command.c_str());
    } else if constexpr (utils::HostOS == utils::OS::Mac) {
      std::string command = "open " + url + " >/dev/null 2>&1";
      system(command.c_str());
    } else if constexpr (utils::HostOS == utils::OS::Windows) {
      std::string command = "start " + url + " > nul";
      system(command.c_str());
    }

    return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace cs::gui::detail
