# GT3 Revisited relay server. Build from the repository root:
#   docker build -f tools/gt3r-relay/Dockerfile -t gt3r-relay .
#   docker run -d --name gt3r-relay --restart unless-stopped -p 47631:47631/udp gt3r-relay
FROM debian:13-slim AS build
RUN apt-get update && apt-get install -y --no-install-recommends g++ cmake make && rm -rf /var/lib/apt/lists/*
COPY src/net /src/src/net
COPY src/base/bytes.h /src/src/base/bytes.h
COPY src/sim/sim_types.h /src/src/sim/sim_types.h
COPY tools/gt3r-relay /src/tools/gt3r-relay
RUN cmake -S /src/tools/gt3r-relay -B /build -DCMAKE_BUILD_TYPE=Release && cmake --build /build -j --target gt3r-relay

FROM debian:13-slim
COPY --from=build /build/gt3r-relay /usr/local/bin/gt3r-relay
RUN useradd --system --no-create-home gt3r
USER gt3r
EXPOSE 47631/udp
ENTRYPOINT ["/usr/local/bin/gt3r-relay"]
