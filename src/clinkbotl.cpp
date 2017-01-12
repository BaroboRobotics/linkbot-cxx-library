// Copyright (c) 2013-2016 Barobo, Inc.
//
// This file is part of liblinkbot.
//
// liblinkbot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// liblinkbot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with liblinkbot.  If not, see <http://www.gnu.org/licenses/>.

#include <linkbot/linkbot.hpp>
#include <linkbot/error.hpp>
#include <math.h>

namespace barobo {

CLinkbotL::CLinkbotL(const std::string& serialId)
: CLinkbot(serialId)
{ 
    // Make sure we are a Linkbot-L
    LinkbotFormFactor form;
    getFormFactor(form);
    if ( form != LINKBOT_FORM_FACTOR_L ) {
        throw Error("Connected Linkbot is not a Linkbot-L.");
    }
}

CLinkbotL::CLinkbotL()
: CLinkbot()
{ 
    // Make sure we are a Linkbot-L
    LinkbotFormFactor form;
    getFormFactor(form);
    if ( form != LINKBOT_FORM_FACTOR_L ) {
        throw Error("Connected Linkbot is not a Linkbot-L.");
    }
}

} // namespace barobo
