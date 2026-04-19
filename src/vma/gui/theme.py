"""Streamlit visual tweaks shared by VMA GUI pages."""
import streamlit as st


def apply_vma_button_theme() -> None:
    """Use light-green emphasis for buttons instead of default primary red."""
    st.markdown(
        """
        <style>
        div[data-testid="stButton"] button {
            background-color: #d4f4d2 !important;
            color: #1a1a1a !important;
            border: 1px solid #8fcf8a !important;
        }
        div[data-testid="stButton"] button:hover {
            background-color: #b8ecb4 !important;
            color: #0d0d0d !important;
            border-color: #6fb86a !important;
        }
        div[data-testid="stButton"] button:focus-visible {
            box-shadow: 0 0 0 2px #a8e6a3 !important;
        }
        div[data-testid="stDownloadButton"] button {
            background-color: #e8f8e6 !important;
            color: #1a1a1a !important;
            border: 1px solid #8fcf8a !important;
        }
        div[data-testid="stDownloadButton"] button:hover {
            background-color: #d4f4d2 !important;
        }
        </style>
        """,
        unsafe_allow_html=True,
    )
