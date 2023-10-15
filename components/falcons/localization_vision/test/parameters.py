# python modules
import logging
import google.protobuf.descriptor

# own modules
import gui


class ParametersProxy:
    """
    Connect sliders with protobuf Params object.
    This class behaves as gui.Parameters, such that the GUI (sliders etc) remains clean.
    """
    # choose composition over inheritance

    def __init__(self, params_proto, range_hints_proto):
        self.params_proto = params_proto
        self.range_hints_proto = range_hints_proto
        self.gui_params = gui.create_standard_gui_params() # system parameters from GUI (zoom, frequency etc)
        self.pb_params = gui.Parameters() # component-specific parameters from protobuf
        self._init_params(params_proto)

    def _init_params(self, proto, parent_name=''):
        # protobuf v3 by default omits fields which have a default value of 0
        # so we cannot just iterate over proto.ListFields(), instead use descriptors
        for descriptor in proto.DESCRIPTOR.fields:
            field_name = descriptor.name
            if descriptor.label == descriptor.LABEL_REPEATED:
                # ignore repeated fields, these cannot easily get sliders assigned to them anyway
                continue
            else:
                value = getattr(proto, field_name)
                name = f'{parent_name}.{field_name}' if parent_name else field_name
                if descriptor.type == descriptor.TYPE_STRING:
                    raise NotImplementedError('strings not yet supported in tuning')
                    # self.add(name, str, None, None, value)
                elif descriptor.type in [descriptor.TYPE_BOOL]:
                    self.pb_params.add(name, bool, False, True, value)
                elif descriptor.type == descriptor.TYPE_MESSAGE:
                    self._init_params(value, name)
                else:
                    # handle numeric types
                    NUMERIC_TYPES = [
                        (int, [descriptor.TYPE_INT32, descriptor.TYPE_INT64, descriptor.TYPE_UINT32, descriptor.TYPE_UINT64, descriptor.TYPE_SINT32, descriptor.TYPE_SINT64]),
                        (float, [descriptor.TYPE_FLOAT, descriptor.TYPE_DOUBLE]),
                    ]
                    for (tt, ptypes) in NUMERIC_TYPES:
                        if descriptor.type in ptypes:
                            if name not in self.range_hints_proto:
                                logging.warning(f'ignoring {name} because of missing tuning range hint')
                                continue
                            slider_range = self.range_hints_proto[name]
                            if slider_range:
                                self.pb_params.add(name, tt, slider_range[0], slider_range[1], value)


    def get(self, name):
        if name in self.gui_params.params:
            return self.gui_params.get(name)
        return self.pb_params.get(name)

    def set(self, name, value):
        if name in self.gui_params.params:
            self.gui_params.set(name, value)
        else:
            self.pb_params.set(name, value)
            self._update_protobuf(name, value)

    def __iter__(self):
        yield from self.gui_params
        yield from self.pb_params

    def __repr__(self):
        gui_repr = '\n'.join([repr(p) for p in self.gui_params])
        pb_repr = '\n'.join([repr(p) for p in self.pb_params])
        return f"{gui_repr}\n{pb_repr}"

    def _update_protobuf(self, name, value):
        names = name.split('.')
        current_proto = self.params_proto
        for n in names[:-1]:
            current_proto = getattr(current_proto, n)
        setattr(current_proto, names[-1], value)

