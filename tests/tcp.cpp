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

#include "baromesh/linkbot.h"
#include <stdio.h>

int main (int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: %s <host> <port>\n", argv[0]);
        return 1;
    }

    printf("Connecting to %s:%s ... ", argv[1], argv[2]);
    if (baromesh::Linkbot* l = linkbotFromTcpEndpoint(argv[1], argv[2])) {
    	printf("Success!\n");

    	char serialId[5];
    	if (!linkbotGetSerialId(l, serialId)) {
    		printf("Serial ID is %s\n", serialId);
    	}
    	else {
    		printf("Failed to get serial ID\n");
    	}

    	linkbotDelete(l);
    }
    else {
    	printf("Failure!\n");
    }
}
