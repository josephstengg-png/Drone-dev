from flight_planner.rag import TemplateStore


def test_template_store_loads_chunks():
    store = TemplateStore()
    assert store.chunks


def test_retrieve_prefers_matching_scan_pattern():
    store = TemplateStore()
    lawnmower_hit = store.retrieve("lawnmower boustrophedon contiguous strip", top_k=1)[0]
    voronoi_hit = store.retrieve("voronoi cell seed partition", top_k=1)[0]
    assert "lawnmower" in lawnmower_hit.heading.lower()
    assert "voronoi" in voronoi_hit.heading.lower()


def test_retrieve_prefers_home_position_chunk_for_planned_home_query():
    store = TemplateStore()
    hit = store.retrieve("plannedHomePosition latitude longitude order", top_k=1)[0]
    assert "home" in hit.heading.lower()


def test_retrieve_falls_back_to_top_chunks_when_nothing_matches():
    store = TemplateStore()
    result = store.retrieve("zzzzznonword qqqqqnomatch", top_k=2)
    assert len(result) == 2
