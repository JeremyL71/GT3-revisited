// How a race is asked for: the GT3R_RACE debug preset (read_preset), course lookup,
// and turning a menu request (resolve_launch) or an online lobby launch (resolve_online)
// into the course, car, event and options the run is built from.
// (Race screen code map: docs/dev/RACE.md.)

#include "app/screen_race_impl.h"

namespace gt3r::race_screen {

DebugPreset read_preset() {
    DebugPreset p;
    const char* env = SDL_getenv("GT3R_RACE");
    if (!env) return p;
    p.have = true;
    std::string s = env;
    size_t pos = 0;
    while (pos < s.size()) {
        size_t end = s.find(',', pos);
        if (end == std::string::npos) end = s.size();
        std::string kv = s.substr(pos, end - pos);
        pos = end + 1;
        size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
        if (k == "course") p.course = v;
        else if (k == "car") p.car = v;
        else if (k == "cam") p.cam = std::atoi(v.c_str());
        else if (k == "auto") p.autodrive = v != "0";
        else if (k == "mt") p.manual = v != "0";
        else if (k == "warp") p.warp = std::atof(v.c_str());
        else if (k == "quit") p.quit = std::atof(v.c_str());
        else if (k == "leave") p.leave = std::atof(v.c_str());
        else if (k == "demo_drive") p.demo_drive = v != "0";
        else if (k == "lic_finish") p.lic_finish = std::atof(v.c_str());
        else if (k == "paint") p.paint = std::atoi(v.c_str());
        else if (k == "hud") p.hud = v != "0";
        else if (k == "perf") p.perf = v != "0";
        else if (k == "gpusync") p.gpusync = v != "0";
        else if (k == "sheen") p.sheen = v != "0";
        else if (k == "pause") p.pause = v != "0";
        else if (k == "prerace") p.prerace = v != "0";
        else if (k == "gtq") p.gt_result = v != "0";
        else if (k == "pause_sel") p.pause = true, p.pause_sel = std::atoi(v.c_str());
        else if (k == "watch") p.watch = v != "0";
        else if (k == "rwarp") p.rwarp = std::atof(v.c_str());
        else if (k == "save") p.save = v != "0";
        else if (k == "ghost") p.ghost = v != "0";
        else if (k == "music") p.music = v != "0";
        else if (k == "replay") p.replay = v;
        else if (k == "event") p.event = v;
        else if (k == "p2") p.two_player = v != "0";
        else if (k == "quick") p.quick = v != "0";
        else if (k == "laps") p.laps = std::atoi(v.c_str());
        else if (k == "exit_laps") p.exit_laps = std::atoi(v.c_str());
        else if (k == "split") p.split = v;
        else if (k == "car2") p.car2 = v;
        else if (k == "qm") p.qm = v;
        else if (k == "ropts") p.pause = true, p.ropts = std::atoi(v.c_str());
        else if (k == "rmode") p.rmode = std::atoi(v.c_str());
        else if (k == "rtarget") p.rtarget = std::atoi(v.c_str());
        else if (k == "hudmode") p.hud_mode = std::atoi(v.c_str());
        else if (k == "ritems") p.ritems = std::atoi(v.c_str());
        else if (k == "rcam") p.rcam = std::atoi(v.c_str());
        else if (k == "lcursor") p.lcursor = float(std::atof(v.c_str()));
        else if (k == "pit") p.pit = v != "0";
        else if (k == "demo") p.demo = v != "0";
        else if (k == "prize") p.prize = std::atoi(v.c_str());
        else if (k == "script") p.script = v;
        else if (k == "tt_event") p.tt_event = v;
        else if (k == "oyaw") p.oyaw = float(std::atof(v.c_str()));
        else if (k == "opitch") p.opitch = float(std::atof(v.c_str()));
        else if (k == "odist") p.odist = float(std::atof(v.c_str()));
        else if (k == "at") p.at = float(std::atof(v.c_str()));
        else if (k == "turn") p.turn = float(std::atof(v.c_str()));
        else if (k == "atkmh") p.at_kmh = float(std::atof(v.c_str()));
    }
    return p;
}

std::string course_path(const Vfs& vfs, const std::string& label) {
    for (const char* region : {"US", "JP", "UK", "EU"}) {
        std::string p = std::string("courses/") + region + "/" + label;
        if (vfs.exists(p)) return p;
    }
    return {};
}

// Online: our car from its descriptor (a garage car runs with its parts and settings).
void RaceScreen::resolve_online() {
    std::optional<game::OnlineCar> oc = game::decode_online_car(net_.car_info);
    if (!oc) return;
    preset_.car = oc->car;
    preset_.paint = oc->paint;
    if (oc->kind != game::OnlineCar::Kind::Garage) return;
    std::string err;
    if (std::optional<sim::CarSpec> spec = game::online_car_spec(db_, *oc, &err)) {
        launch_spec_ = std::make_shared<const sim::CarSpec>(std::move(*spec));
        launch_.request.car_wheel = oc->wheel;
        launch_.request.racing_gearbox = game::online_car_racing_gearbox(db_, *oc);
    } else {
        LOG_WARN("Online: garage car %s: %s", oc->car.c_str(), err.c_str());
    }
}

// Maps the menu's identifiers to a course label and a database car label:
// course = course label (or taken from the race code's event), car = car
// label or an arcade/opponent variant label (-> its base car).
void RaceScreen::resolve_launch() {
    // Arcade time trial ("compulsory"): course ids tt01..tt10 are the
    // time-attack events AT010001..AT100001 (course + a fixed car variant).
    const menu::flow::GameRequest& rq = launch_.request;
    std::string code = rq.event.empty() ? rq.race_code : rq.event;
    if (rq.course.size() == 4 && rq.course.rfind("tt", 0) == 0) code = "AT" + rq.course.substr(2) + "0001";
    const data::Event* ev = db_.event(code);
    std::string course = launch_.request.course;
    if ((course.empty() || course_path(*app_->vfs(), course).empty()) && ev) course = ev->course;
    if (course.empty() || course_path(*app_->vfs(), course).empty()) course = "trialmt";
    // Menu car ids are database labels without their leading '_'; variants map to their base car.
    auto base_car = [&](const std::string& id) -> std::string {
        for (const std::string& l : {id, "_" + id}) {
            if (db_.car(l)) return l;
            if (const data::CarVariant* v = db_.variant(l)) return v->car;
            for (const data::CarVariant& v : db_.arcade_cars())
                if (v.label == l && db_.car(v.car)) return v.car;
        }
        return {};
    };
    std::string car = base_car(launch_.request.car);
    if (car.empty() && ev && !ev->entries.empty()) car = base_car(ev->entries[0]);
    if (car.empty()) car = "r34_gtr_v_99";
    preset_.course = course;
    preset_.car = car;
    // Courses the menu offers that are not in the database course list (e.g. licence layouts).
    bool listed = false;
    for (const CourseEntry& c : courses_) listed = listed || c.label == course;
    if (!listed) courses_.push_back({course, course});
    // Races with opponents: the event gives the field, laps and AI parameters.
    if ((rq.mode == "single_race" || rq.mode == "go_race") && db_.event(code)) race_event_ = code;
    if (!rq.opponents.empty()) race_opponents_ = rq.opponents; // GT mode: the field drawn by the menus
    // Arcade Single Race: the field is drawn from the event's pool with the race module's
    // generator (gt_mode §20.4), slots 1 .. cars-1.
    else if (rq.mode == "single_race" && ev && !online_)
        race_opponents_ = race::draw_opponents(db_, *ev, std::max(int(ev->cars_in_race), 1) - 1, race_info_random());
    // The arcade time trial: one timed lap, whatever the event's lap count (presentation §B7).
    if (rq.mode == "time_trial" && ev) race_laps_ = race::kTimeTrialLaps;
    // Time attack / licence tests: a flying start when the event gives one.
    if (ev && (rq.mode == "time_trial" || rq.mode == "go_license" || rq.mode == "go_machine_test"))
        tt_start_ = event_start(*ev);
    if (ev && rq.mode == "go_license") license_event_ = code;
    // Licence tests drive the event's entry 0 in the event's slot-0 colour (+0x110, a carcolor id;
    // 0xFFFFFFFF = none), not a garage paint.
    // (Quick race practice with a car of the player's choice: that car's own paint.)
    event_paint_id_ = ev && rq.mode == "go_license" && (!rq.practice || rq.car.empty()) && ev->bytes.size() >= 0x114
                          ? rd32(ev->bytes, 0x110)
                          : 0xFFFFFFFFu;
    // Title-screen attract demo: the event's cars all driven by the computer, TV cameras,
    // no HUD; any button ends it.
    std::string attract_slot0;
    if (rq.mode == "attract_demo" && ev) {
        attract_ = true;
        race_event_ = code;
        preset_.autodrive = true;
        preset_.hud = false;
        preset_.cam = int(CameraMode::Tv);
        // The whole field from the event's pool (gt_mode §20.4): the demo's
        // race information has no player car (gcf section 12 word +8 is "", whose label
        // hash is 0), so every slot 0 .. cars-1 is drawn (gt_mode §15)
        // on the race module's state; slot 0 is the car the demo follows.
        // (A demo with its own car keeps it in slot 0 and draws slots 1 .. cars-1.)
        const bool own = !rq.car.empty();
        std::vector<std::string> field =
            race::draw_opponents(db_, *ev, std::max(int(ev->cars_in_race), 1) - (own ? 1 : 0), race_info_random());
        if (own) field.insert(field.begin(), rq.car);
        if (!field.empty()) {
            attract_slot0 = field[0];
            race_opponents_.assign(field.begin() + 1, field.end());
            if (std::string c = base_car(field[0]); !c.empty()) preset_.car = car = c;
        }
    }
    // Machine tests (presentation §B7): the event's lap-count byte is the test (1 = 0-400 m,
    // 2 = 0-1000 m, 3 = top speed); they run no laps.
    if (rq.mode == "go_machine_test")
        machine_.emplace(ev ? int(ev->laps) : code == "M1000" ? 2 : code == "MMAX" ? 3 : 1);
    // The engine condition the runs start with and hand back (physics §10.3; GT mode fills it
    // for the modes that carry it), and the garage car's dirt.
    has_condition_ = rq.car_condition;
    start_condition_ = run_condition_ = rq.car_condition ? sim::EngineCondition{rq.car_odometer, rq.car_wear_m, rq.car_oil_m}
                                                         : sim::EngineCondition{};
    live_condition_ = false;
    car_dirt_ = 0;
    dirt_dims_ = true;
    if (rq.car_spec)
        if (game::GtModeSession* gs = game::gt_mode_session())
            if (const game::GarageCar* gc = gs->gt().state().ride_car()) car_dirt_ = gc->cond.dirt;
    if (rq.car_spec) launch_spec_ = rq.car_spec;
    // Licence / machine tests: the event's entry-0 car is a variant with its own parts
    // (Quick race practice: only when no other car was chosen).
    else if (ev && !ev->entries.empty() &&
             (rq.mode == "go_license" || rq.mode == "go_machine_test" || !theater_demo_.empty()) &&
             (!rq.practice || rq.car.empty()))
        // (also the theater's time-trial demonstrations: the event's car with its parts)
        if (std::optional<sim::CarSpec> vs = sim::build_car_spec(db_, ev->entries[0]))
            launch_spec_ = std::make_shared<const sim::CarSpec>(std::move(*vs));
    // The attract demo's slot 0: the drawn entry with its parts.
    if (!attract_slot0.empty())
        if (std::optional<sim::CarSpec> vs = sim::build_car_spec(db_, attract_slot0))
            launch_spec_ = std::make_shared<const sim::CarSpec>(std::move(*vs));
    // The theater's demonstrations: the car the run was recorded with (the demo's car record,
    // parts and settings): the arcade time trials' dirt tyres (stage 12) differ from the
    // event car's.
    if (!theater_demo_.empty())
        if (std::optional<Bytes> b = app_->vfs()->read("demo/" + theater_demo_))
            if (std::optional<DemoReplay> d = DemoReplay::parse(*b); d && !d->car_record.empty()) {
                std::string dc;
                std::array<u64, 27> parts{};
                sim::CarSettings st;
                int ref_final = 0;
                game::decode_car_equipments(d->car_record, db_, dc, parts, st, ref_final);
                std::string err;
                if (!dc.empty())
                    if (std::optional<sim::CarSpec> ds = sim::build_car_spec(db_, dc, parts, st, &err))
                        launch_spec_ = std::make_shared<const sim::CarSpec>(std::move(*ds));
            }
    // Arcade cars (table 35 variants) run with their own parts; the tyres follow the car page's
    // RACING / DRIFT choice: GT3 switches both to the car's stage-10 (racing, "_k") or stage-11
    // (drift, "_l") records; dirt courses force stage 12 ("_m"), the wet Route 5 stage + 13.
    if (!rq.car_spec && theater_demo_.empty()) {
        int tire_mode = 0;
        if constexpr (requires { rq.tire_mode; }) tire_mode = std::max(0, rq.tire_mode);
        std::string err;
        if (std::optional<ArcadeCar> ac = arcade_car(db_, launch_.request.car, tire_mode, preset_.course, &err)) {
            launch_spec_ = std::make_shared<const sim::CarSpec>(std::move(ac->spec));
            launch_tire_stage_ = ac->stage;
            LOG_INFO("Race: arcade car %s, tyres stage %d (%s)", ac->label.c_str(), ac->stage,
                     tire_mode == 1 ? "drift" : "racing");
        }
    }
    if (!rq.car_model.empty()) launch_model_ = rq.car_model;
    preset_.paint = std::max(0, rq.car_color);
    // The options' default gearbox (free run, time trial, licence, machine test, GT mode);
    // arcade single race and 2P use the car page's AUTOMATIC / MANUAL instead.
    if (rq.mode != "single_race" && rq.mode != "two_player_battle")
        preset_.manual = menu::flow::option_value(&app_->config(), "gear_auto") == "off";
    // Options ASM / TCS: arcade single race, free run and 2P battle only.
    aids_options_apply_ = rq.mode == "single_race" || rq.mode == "free_run" || rq.mode == "two_player_battle";
    asm_on_ = menu::flow::option_value(&app_->config(), "assist_asm") != "off";
    tcs_on_ = menu::flow::option_value(&app_->config(), "assist_tcs") != "off";
    // Arcade car page: AUTOMATIC / MANUAL (-1 = not chosen: keep the setting).
    if constexpr (requires { rq.transmission; })
        if (rq.transmission >= 0) preset_.manual = rq.transmission == 1;
    if (launch_.request.mode == "two_player_battle") {
        two_player_ = true;
        if (db_.event(code)) race_event_ = code; // A2P0001: laps
        // Player 2's car page: car, paint, AT/MT and RACING/DRIFT (own pad).
        launch_spec2_.reset();
        car2_paint_ = -1, manual2_ = -1;
        if constexpr (requires { rq.car2; }) {
            if (!rq.car2.empty()) {
                std::string c2 = base_car(rq.car2);
                if (!c2.empty()) preset_.car2 = c2;
                std::string err2;
                if (rq.car2_spec) { // a garage car: its fitted parts and settings
                    launch_spec2_ = rq.car2_spec;
                    LOG_INFO("Race: player 2 garage car %s", rq.car2.c_str());
                } else if (std::optional<ArcadeCar> ac = arcade_car(db_, rq.car2, std::max(0, rq.tire_mode2), course, &err2)) {
                    launch_spec2_ = std::make_shared<const sim::CarSpec>(std::move(ac->spec));
                    LOG_INFO("Race: player 2 arcade car %s, tyres stage %d", ac->label.c_str(), ac->stage);
                }
            }
            car2_paint_ = rq.car2_color;
            if (rq.transmission2 >= 0) manual2_ = rq.transmission2 == 1 ? 1 : 0;
        }
    }
    LOG_INFO("Race: menu request %s (code %s, course '%s', car '%s') -> %s / %s", launch_.request.mode.c_str(),
             launch_.request.race_code.c_str(), launch_.request.course.c_str(), launch_.request.car.c_str(), course.c_str(), car.c_str());
}

} // namespace gt3r::race_screen
