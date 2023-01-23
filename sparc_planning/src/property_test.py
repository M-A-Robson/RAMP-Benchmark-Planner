from al_structures import *

def test_sort_property():
    assert(Property('a','b',Relation.IS_OF_SORT).to_string() == '#a(b)')

def test_sort_equality():
    assert(Property('a','b',Relation.EQUAL).to_string() == 'a=b')
