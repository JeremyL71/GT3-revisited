#include "menu/flow/game_hooks.h"

namespace gt3r::menu::flow {

namespace {
GameRequestHandler& handler() {
    static GameRequestHandler h;
    return h;
}
std::optional<GameResult>& result() {
    static std::optional<GameResult> r;
    return r;
}
} // namespace

void post_game_result(GameResult r) { result() = std::move(r); }
std::optional<GameResult> take_game_result() {
    std::optional<GameResult> r = std::move(result());
    result().reset();
    return r;
}

void set_game_request_handler(GameRequestHandler h) { handler() = std::move(h); }
const GameRequestHandler& game_request_handler() { return handler(); }

} // namespace gt3r::menu::flow
