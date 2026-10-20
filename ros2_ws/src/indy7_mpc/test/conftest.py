"""Keep one Qt application alive while GUI fixtures create/destroy windows."""

import os

import pytest


@pytest.fixture(scope="session")
def qapp():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    widgets = pytest.importorskip("PyQt5.QtWidgets")
    app = widgets.QApplication.instance() or widgets.QApplication([])
    # A session reference prevents cyclic GUI callbacks from collecting an
    # earlier QApplication while later fixtures already own new windows.
    yield app
